#include "geometry_msgs/Point.h"
#include "multi_lidar_calibration/ml_calib.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "bag2pcd/bag2pcd.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Transform.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/make_shared.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include "patchwork/patchworkpp.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <dynamic_reconfigure/server.h>
#include <ml_calib/ml_calibConfig.h>
#include "ros/time.h"
#include "visualization_msgs/Marker.h"
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <visualization_msgs/MarkerArray.h>

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using CloudPtr = pcl::shared_ptr<PointCloud>;
using ConstCloudPtr = pcl::shared_ptr<const PointCloud>;
using OctreeT = pcl::octree::OctreePointCloud<PointT>;
namespace fs = std::filesystem;

ml_calib::ml_calibConfig ml_calib_config;
std::mutex mtx_config;
bool config_changed = true;
patchwork::Params ml_calib::MLiDAR::patchwork_parameters;

/* template<typename PointT>
double checkPlane(pcl::shared_ptr<const pcl::PointCloud<PointT>> cloud, const pcl::Indices& indices, Eigen::Vector3f& plane_normal)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices,centroid);
    Eigen::Matrix3f covariance_matrix;
    pcl::computeCovarianceMatrixNormalized(*cloud, indices, centroid, covariance_matrix);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix);
    Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();      // 特征值（升序排列：λ0 ≤ λ1 ≤ λ2）
    Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();    // 对应特征向量
    plane_normal = eigenvectors.col(0);  // 最小特征值 λ0 对应的特征向量为法向量

    return 2 * (eigenvalues(1) - eigenvalues(0)) / (eigenvalues.sum());
} */

template<typename PointT>
pcl::shared_ptr<pcl::PointCloud<PointT>> pcdSector(pcl::shared_ptr<pcl::PointCloud<PointT>> pcd, float begin_angle, float end_angle, std::vector<int>& indices)
{
    pcl::shared_ptr<pcl::PointCloud<PointT>> ret = pcl::make_shared<pcl::PointCloud<PointT>>();

    if (pcd.get() == nullptr) return ret;

    for (int i = 0; i < pcd->size(); i++)
    {
        PointT p = pcd->at(i);
        if (pcl::isFinite(p)){
            Eigen::Affine3f affine(Eigen::AngleAxisf(DEG2RAD(-begin_angle), Eigen::Vector3f::UnitZ()));
            PointT transed_p = pcl::transformPoint(p, affine);
            float angle = RAD2DEG(std::atan2(transed_p.y, transed_p.x));
            if (angle < 0) angle += 360;
            if (angle >=0  && angle < end_angle - begin_angle){
                ret->push_back(p);
                indices.push_back(i);
            }
        }
    }
    return ret;
}

/* pcl::shared_ptr<pcl::PointCloud<PointT>> pcdDistanceRejectPoint(pcl::shared_ptr<pcl::PointCloud<PointT>> pcd, float begin_distance, float end_distance, std::vector<int>& indices)
{
    pcl::shared_ptr<pcl::PointCloud<PointT>> ret = pcl::make_shared<pcl::PointCloud<PointT>>();
    if (pcd.get() == nullptr) return ret;
    for (int i = 0; i < pcd->size(); i++)
    {
        PointT p = pcd->at(i);
        if (pcl::isFinite(p)){
            float dist = std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2);
            if (dist < std::pow(end_distance, 2) && dist > std::pow(begin_distance, 2)){
                ret->push_back(p);
                indices.push_back(i);
            }
        }
    }
    return ret;
} */


/* void searchInterSection(ConstCloudPtr pcd0, 
                        OctreeT pcd0_octree, 
                        ConstCloudPtr pcd1, 
                        const pcl::KdTreeFLANN<PointT>& pcd1_kd,
                        double threshould,
                        std::vector<std::pair<int, int>>& idx_pairs)
{
    idx_pairs.clear();
    std::cout << "leaf size: " << pcd0_octree.getLeafCount() << std::endl;
    for (auto leaf_it = pcd0_octree.leaf_begin(); leaf_it != pcd0_octree.leaf_end(); ++leaf_it) {
        std::vector<int> point_indices;
        leaf_it.getLeafContainer().getPointIndices(point_indices);

        float min_dist = INFINITY;
        int idx_pcd1(-1), idx_pcd0(-1);
        for (int idx : point_indices) {
            PointT pt = pcd0->points[idx];

            if (pcl::isFinite(pt)){
                pcl::Indices indices;
                std::vector<float> distance;
                pcd1_kd.nearestKSearch(pt, 1, indices, distance);
                if (distance[0] < min_dist && distance[0] < threshould){
                    min_dist = distance[0];
                    idx_pcd1 = indices[0];
                    idx_pcd0 = idx;
                }
            }
        }
        if (idx_pcd1 != -1){
            idx_pairs.push_back(std::make_pair(idx_pcd0, idx_pcd1));
        }
    }
}
 */

/* void visualizeIntersection(pcl::IndicedMerge<PointT>& merged_back, pcl::KdTreeFLANN<PointT>& kd_merged_back, std::vector<std::pair<int, int>>& idx_pairs,
                          const std::string str0, const std::string str1, float threshould)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    for (auto [id0, id1] : idx_pairs)
    {
        int id0_ = merged_back.idxLocal2Global(str0, id0);
        int id1_ = merged_back.idxLocal2Global(str1, id1);
        std::cout << id0 << " " << id1 << std::endl;
        pcl::Indices indices;
        std::vector<float> dists;
        kd_merged_back.radiusSearch(merged_back.at(id0_), 0.8, indices, dists);

        Eigen::Vector3f normal;
        if (threshould < checkPlane<PointT>(merged_back.merged_cloud, indices, normal)){
            pcl::PointXYZ l0_begin;
            pcl::copyPoint<PointT, pcl::PointXYZ>(merged_back.at(id0_), l0_begin);
            Eigen::Vector3f end_val = normal.cast<float>() + l0_begin.getVector3fMap();
            pcl::PointXYZ l0_end(end_val.x(), end_val.y(), end_val.z());
            viewer->addLine<pcl::PointXYZ>(l0_begin, l0_end, 1.0, 0.0, 0.0, std::to_string(id0_));

            pcl::PointXYZ l1_begin;
            pcl::copyPoint<PointT, pcl::PointXYZ>(merged_back.at(id1_), l1_begin);
            end_val = normal.cast<float>() + l1_begin.getVector3fMap();
            pcl::PointXYZ l1_end(end_val.x(), end_val.y(), end_val.z());
            viewer->addLine<pcl::PointXYZ>(l1_begin, l1_end, 0, 1, 0, std::to_string(id1_));
        }
    }
    viewer->addPointCloud<PointT>(merged_back.merged_cloud, "back_merged");
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
} */


template<typename PointT>
pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud(const pcl::PointCloud<PointT>& in, const Eigen::Vector3i& rgb)
{
    pcl::PointCloud<pcl::PointXYZRGB> out;
    pcl::copyPointCloud(in, out);
    for(auto& p: out){
       p.r = rgb.x();
       p.g = rgb.y();
       p.b = rgb.z();
    }
    return out;
}



/* void segGround(ConstCloudPtr pcd, std::unordered_set<int>& result, float z_threshold = 1)
{
    result.clear();
    patchwork::Params patchwork_parameters;
    patchwork_parameters.verbose = ml_calib_config.verbose;
    patchwork_parameters.enable_RNR = ml_calib_config.RNR;
    patchwork_parameters.enable_RVPF = ml_calib_config.RVPF;
    patchwork_parameters.enable_TGR  = ml_calib_config.TGR;

    patchwork_parameters.num_iter              = ml_calib_config.num_iter;
    patchwork_parameters.num_lpr               = ml_calib_config.num_lpr;
    patchwork_parameters.num_min_pts           = ml_calib_config.num_min_pts;
    patchwork_parameters.num_zones             = ml_calib_config.num_zones;
    patchwork_parameters.num_rings_of_interest = ml_calib_config.num_rings_of_interest;

    patchwork_parameters.RNR_ver_angle_thr     = ml_calib_config.RNR_ver_angle_thr;
    patchwork_parameters.RNR_intensity_thr     = ml_calib_config.RNR_intensity_thr;

    patchwork_parameters.sensor_height         = ml_calib_config.sensor_height;
    patchwork_parameters.th_seeds              = ml_calib_config.th_seeds;
    patchwork_parameters.th_dist               = ml_calib_config.th_dist;
    patchwork_parameters.th_seeds_v            = ml_calib_config.th_seeds_v;
    patchwork_parameters.th_dist_v             = ml_calib_config.th_dist_v;

    patchwork_parameters.max_range             = ml_calib_config.max_range;
    patchwork_parameters.min_range             = ml_calib_config.min_range;
    patchwork_parameters.uprightness_thr       = ml_calib_config.uprightness_thr;
    patchwork_parameters.adaptive_seed_selection_margin = ml_calib_config.adaptive_seed_selection_margin;

    patchwork_parameters.max_flatness_storage  = ml_calib_config.max_flatness_storage;
    patchwork_parameters.max_elevation_storage = ml_calib_config.max_elevation_storage;

    patchwork::PatchWorkpp patchworkpp(patchwork_parameters);
   
    Eigen::MatrixXf cloud;
    cloud.resize(pcd->size(), 4);
    for (int i = 0; i < pcd->size(); i++)
    {
        cloud.row(i) << pcd->at(i).x, pcd->at(i).y, pcd->at(i).z, pcd->at(i).intensity;
    }
    patchworkpp.estimateGround(cloud);
    Eigen::VectorXi ground_idx    = patchworkpp.getGroundIndices();
    Eigen::VectorXi nonground_idx = patchworkpp.getNongroundIndices();

    for (int i = 0; i < ground_idx.size(); i++)
    {
        if (pcd->at(ground_idx(i)).z > z_threshold)
        {
            continue;
        }
        result.insert(ground_idx(i));
    }
} */

void updatePatchWorkParameters()
{
    ros::spinOnce();
    auto& patchwork_para = ml_calib::MLiDAR::patchwork_parameters;
    patchwork_para.verbose = ml_calib_config.verbose;
    patchwork_para.enable_RNR  = ml_calib_config.RNR;
    patchwork_para.enable_RVPF = ml_calib_config.RVPF;
    patchwork_para.enable_TGR  = ml_calib_config.TGR;

    patchwork_para.num_iter              = ml_calib_config.num_iter;
    patchwork_para.num_lpr               = ml_calib_config.num_lpr;
    patchwork_para.num_min_pts           = ml_calib_config.num_min_pts;
    patchwork_para.num_zones             = ml_calib_config.num_zones;
    patchwork_para.num_rings_of_interest = ml_calib_config.num_rings_of_interest;

    patchwork_para.RNR_ver_angle_thr = ml_calib_config.RNR_ver_angle_thr;
    patchwork_para.RNR_intensity_thr = ml_calib_config.RNR_intensity_thr;

    patchwork_para.sensor_height      = ml_calib_config.sensor_height;
    patchwork_para.th_seeds          = ml_calib_config.th_seeds;
    patchwork_para.th_dist           = ml_calib_config.th_dist;
    patchwork_para.th_seeds_v        = ml_calib_config.th_seeds_v;
    patchwork_para.th_dist_v         = ml_calib_config.th_dist_v;
    patchwork_para.max_range         = ml_calib_config.max_range;
    patchwork_para.min_range         = ml_calib_config.min_range;
    patchwork_para.uprightness_thr   = ml_calib_config.uprightness_thr;
    patchwork_para.adaptive_seed_selection_margin = ml_calib_config.adaptive_seed_selection_margin;
}

void constructMarker(visualization_msgs::MarkerArray& mark, std::vector<ml_calib::MLiDARConstraintDetail>& details, CloudPtr pcd0, CloudPtr pcd1, pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> intersected_pcd, int type)
{
    int pre_id = mark.markers.size();
    for (int i = 0; i < details.size(); i++)
    {
        auto detail = details[i];
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << detail.distance; // 保留2位小数
        visualization_msgs::Marker marker_text;

        marker_text.header.frame_id = "lidar";
        marker_text.header.stamp = ros::Time::now();
        marker_text.ns = "constraint";
        marker_text.id = i + pre_id;

        marker_text.action = visualization_msgs::Marker::MODIFY;


        marker_text.pose.orientation.x = 0;
        marker_text.pose.orientation.y = 0;
        marker_text.pose.orientation.z = 0;
        marker_text.pose.orientation.w = 1;

        marker_text.color.a = 1.0f;
        marker_text.lifetime = ros::Duration(1);
        if (type == 0){
            marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_text.text = oss.str();
            float scale = 0.4;
            Eigen::Vector3d tangent1;
            if (std::abs(detail.normal.z()) < 0.9)
                tangent1 = detail.normal.cross(Eigen::Vector3d::UnitZ());
            else
                tangent1 = detail.normal.cross(Eigen::Vector3d::UnitY());

            tangent1.normalize();  // 归一化
            tangent1 *= scale;  // 缩放长度
            Eigen::Vector3d tangent2 = detail.normal.cross(tangent1);
            tangent2.normalize();
            tangent2 *= scale;  // 缩放长度
            auto tangent = tangent1 + tangent2 + detail.normal * scale * 0.5;

            marker_text.pose.position.x = pcd0->at(detail.id0).x + tangent.x();
            marker_text.pose.position.y = pcd0->at(detail.id0).y + tangent.y();
            marker_text.pose.position.z = pcd0->at(detail.id0).z + tangent.z();

            if (detail.distance < 0.1){
                marker_text.color.r = 0.0f;
                marker_text.color.g = 1.0f;
                marker_text.color.b = 0.0f;
            }else{
                marker_text.color.r = 1.0f;
                marker_text.color.g = 0.0f;
                marker_text.color.b = 0.0f;
            }
            marker_text.scale.x = 0.6;
            marker_text.scale.y = 0.6;
            marker_text.scale.z = 0.6;
        }else {
            marker_text.type = visualization_msgs::Marker::LINE_STRIP;
            geometry_msgs::Point p0_, p1_;
            p0_.x = pcd0->at(detail.id0).x;
            p0_.y = pcd0->at(detail.id0).y;
            p0_.z = pcd0->at(detail.id0).z;
            p1_.x = p0_.x + detail.normal.x();
            p1_.y = p0_.y + detail.normal.y();
            p1_.z = p0_.z + detail.normal.z();
            marker_text.points.push_back(p0_);
            marker_text.points.push_back(p1_);
            marker_text.scale.x = 0.02;
            marker_text.scale.y = 0.01;
            marker_text.scale.z = 0.01;
            if (detail.label == "ground"){
                marker_text.color.r = 1.0f;
                marker_text.color.g = 0.0f;
                marker_text.color.b = 0.0f;
            }else{
                marker_text.color.r = 1.0f;
                marker_text.color.g = 0.5f;
                marker_text.color.b = 0.0f;               
            }
        }
        mark.markers.push_back(marker_text);

        pcl::PointXYZRGBA p0, p1;
        pcl::copyPoint<PointT, pcl::PointXYZRGBA>(pcd0->at(detail.id0), p0);
        pcl::copyPoint<PointT, pcl::PointXYZRGBA>(pcd1->at(detail.id1), p1);
        if (detail.label == "ground") {
            p0.r = 255; p0.g = 0; p0.b = 0; p0.a = 100;
            p1.r = 255; p1.g = 0; p1.b = 0; p1.a = 100;
        }else{
            p0.r = 255; p0.g = 125; p0.b = 0; p0.a = 100;
            p1.r = 255; p1.g = 125; p1.b = 0; p1.a = 100;
        }
        intersected_pcd->push_back(p0);
        intersected_pcd->push_back(p1);
    }
}

void callbackConfig(ml_calib::ml_calibConfig &config, uint32_t level) {
    std::cout << "Reconfigure Request: " << std::endl;
    std::lock_guard<std::mutex> lock(mtx_config);
    ml_calib_config = config;
    config_changed = true;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ml_calib");
    ros::NodeHandle nh("~");
    std::string project_path, bag_folder, launch_cmd_dir, config_url, g3reg_config_path;
    nh.getParam("project_path", project_path);
    nh.getParam("bag_folder_path", bag_folder);
    nh.getParam("launch_command_dir", launch_cmd_dir);

    std::string topic_pub_front("/ml_calib/bag_front_f"), topic_pub_left("/ml_calib/bag_front_l"), topic_pub_right("/ml_calib/bag_front_r"), topic_pub_back("/ml_calib/bag_front_b");
    ros::Publisher pub_front_bag_f = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_front, 10);
    ros::Publisher pub_front_bag_l = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_left, 10);
    ros::Publisher pub_front_bag_r = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_right, 10);
    ros::Publisher pub_front_bag_b = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_back, 10);


    std::string topic_pub2_front("/ml_calib/bag_back_f"), topic_pub2_left("/ml_calib/bag_back_l"), topic_pub2_right("/ml_calib/bag_back_r"), topic_pub2_back("/ml_calib/bag_back_b");
    ros::Publisher pub_back_bag_f = nh.advertise<sensor_msgs::PointCloud2>(topic_pub2_front, 10);
    ros::Publisher pub_back_bag_l = nh.advertise<sensor_msgs::PointCloud2>(topic_pub2_left, 10);
    ros::Publisher pub_back_bag_r = nh.advertise<sensor_msgs::PointCloud2>(topic_pub2_right, 10);
    ros::Publisher pub_back_bag_b = nh.advertise<sensor_msgs::PointCloud2>(topic_pub2_back, 10);

    ros::Publisher pub_constraint_front = nh.advertise<visualization_msgs::MarkerArray>("/ml_calib/constraint_front", 10);
    ros::Publisher pub_constraint_back = nh.advertise<visualization_msgs::MarkerArray>("/ml_calib/constraint_back", 10);

    ros::Publisher pub_normal_front = nh.advertise<visualization_msgs::MarkerArray>("/ml_calib/normal_front", 10);
    ros::Publisher pub_normal_back = nh.advertise<visualization_msgs::MarkerArray>("/ml_calib/normal_back", 10);

    ros::Publisher pub_intersection_front = nh.advertise<sensor_msgs::PointCloud2>("/ml_calib/intersection_front", 10);
    ros::Publisher pub_intersection_back = nh.advertise<sensor_msgs::PointCloud2>("/ml_calib/intersection_back", 10);

    std::string topic_pub_groud_front("/ml_calib/groud_front"), topic_pub_groud_back("/ml_calib/groud_back");
    ros::Publisher pub_groud_front = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_groud_front, 10);
    ros::Publisher pub_groud_back = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_groud_back, 10);

    g3reg_config_path = fs::path(project_path) / "third_party/G3Reg/configs";
    config_url = fs::path(project_path) / "config/multi_scene_calib.yaml";

    YAML::Node root = YAML::LoadFile(config_url);

    bool recalibration_mode;
    nh.getParam("recalibration_mode", recalibration_mode);
    std::vector<std::string> lidar_topics = root["lidar_topics"].as<std::vector<std::string>>();
    std::vector<std::string> bag_file_lst = root["bag_files"].as<std::vector<std::string>>();

    boost::shared_ptr<dynamic_reconfigure::Server<ml_calib::ml_calibConfig>> server_f(new dynamic_reconfigure::Server<ml_calib::ml_calibConfig>(nh));

    server_f->setCallback(callbackConfig);

    fs::path bag_path = fs::path(launch_cmd_dir) / fs::path(bag_folder);

    std::vector<std::vector<CloudPtr>> pcd_lst;

    for(const auto bag_file: bag_file_lst)
    {
        fs::path bag_url = bag_path / bag_file;
        std::vector<CloudPtr>& bag_clouds = pcd_lst.emplace_back();
        if (fs::exists(bag_url)){
            for (const std::string& topic : lidar_topics)
            {
                std::cout << "Try open bagfile: " << bag_url << " " << topic << std::endl;        
                std::vector<CloudPtr> pcd;
                bag2pcd::rosbag2PointCloud<PointT>(bag_url, topic, pcd);
                bag_clouds.push_back(pcd[0]);
            }
        }else{
            std::cout << "File doesn't exist: " << bag_url << std::endl;        
            exit(0);
        }
    }

    Eigen::IOFormat eigen_fmt(-1, 0, ", ", ", \n");

    
    Eigen::Affine3d mat_front(Eigen::Affine3d::Identity());
    Eigen::Affine3d mat_left(Eigen::Affine3d::Identity());
    Eigen::Affine3d mat_right(Eigen::Affine3d::Identity());
    Eigen::Affine3d mat_back(Eigen::Affine3d::Identity());

    updatePatchWorkParameters();
    ml_calib::MLCalib calib(fs::path(g3reg_config_path) / "apollo_lc_bm/fpfh_pagor.yaml");

    if (lidar_topics.size() == 4){
        calib.addLiDAR("front");
        calib.addLiDAR("left");
        calib.addLiDAR("right");
        calib.addLiDAR("back");


        auto getPrePose = [&](const std::string& name){
            std::vector<double> pre_mat_vec = root[name].as<std::vector<double>>();
            if (pre_mat_vec.size() != 16){
                std::cout << "Pre-mat size is not 16, please check the config file." << std::endl;
                exit(0);
            }
            Eigen::Matrix4d pose = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(pre_mat_vec.data());;
            return pose;
        };
        int fl_id(-1), fr_id(-1), lr_id(-1), bl_id(-1), br_id(-1);
        if (recalibration_mode){
            calib["front"]->push_back("bag_front", pcd_lst[0][0]);
            calib["left"]->push_back("bag_front", pcd_lst[0][1]);
            calib["right"]->push_back("bag_front", pcd_lst[0][2]);
            calib["back"]->push_back("bag_front", pcd_lst[0][3]);

            calib["front"]->push_back("bag_back", pcd_lst[1][0]);
            calib["left"]->push_back("bag_back", pcd_lst[1][1]);
            calib["right"]->push_back("bag_back", pcd_lst[1][2]);
            calib["back"]->push_back("bag_back", pcd_lst[1][3]);

            Eigen::Matrix4d front_lidar_pose = getPrePose("front_pose");
            calib.setLidarPose("front", front_lidar_pose);
            calib.setRootLidar("front");
            calib.setRootLidarHeight(root["front_lidar_height"].as<double>());
            calib.setLidarFixed("front", true);

            pcl::Indices indices;
            CloudPtr sectored_left_pcd = pcdSector<PointT>(calib["left"]->source_pcds["bag_front"], -180, 0, indices);
            indices.clear();
            CloudPtr sectored_right_pcd = pcdSector<PointT>(calib["right"]->source_pcds["bag_front"], 0, 180, indices);

            calib["left"]->push_back("bag_front_sector", sectored_left_pcd);
            calib["right"]->push_back("bag_front_sector", sectored_right_pcd);

            calib["front"]->addTargetLeaf("bag_front", calib["left"], "bag_front_sector");
            calib["front"]->addTargetLeaf("bag_front", calib["right"], "bag_front_sector");

            calib.updateInitPose();

            CloudPtr merged_back = calib["front"]->getPCDInGlobal("bag_back");
            *merged_back += *calib["left"]->getPCDInGlobal("bag_back");
            *merged_back += *calib["right"]->getPCDInGlobal("bag_back");

            CloudPtr merged_front = calib["front"]->getPCDInGlobal("bag_front");
            *merged_front += *calib["left"]->getPCDInGlobal("bag_front");
            *merged_front += *calib["right"]->getPCDInGlobal("bag_front");

            pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test5.pcd", *merged_front);
            pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test0.pcd", *merged_back);

            pcl::transformPointCloud(*merged_back, *merged_back, front_lidar_pose.inverse().cast<float>());  

            calib["front"]->push_back("merged_back", merged_back);
            calib["front"]->addTargetLeaf("merged_back", calib["back"], "bag_back");
            calib.updateInitPose();

            fl_id = calib.addConstraint("front", "bag_front", "left", "bag_front", ml_calib_config.fr_weight);
            fr_id = calib.addConstraint("front", "bag_front", "right", "bag_front", ml_calib_config.fr_weight);
            lr_id = calib.addConstraint("left", "bag_front", "right", "bag_front", ml_calib_config.lr_weight);
            bl_id = calib.addConstraint("left", "bag_back", "back", "bag_back",  ml_calib_config.bl_weight);
            br_id = calib.addConstraint("right", "bag_back", "back", "bag_back",  ml_calib_config.br_weight);
        }else{
            bool fix_front = root["fix_front"].as<bool>();
            calib.setLidarPose("front", getPrePose("front_pose"));
            calib.setLidarFixed("front", fix_front);

            bool fix_left = root["fix_left"].as<bool>();
            calib.setLidarPose("left", getPrePose("left_pose"));
            calib.setLidarFixed("left", fix_left);

            bool fix_right = root["fix_right"].as<bool>();
            calib.setLidarPose("right", getPrePose("right_pose"));
            calib.setLidarFixed("right", fix_right);

            bool fix_back = root["fix_back"].as<bool>();
            calib.setLidarPose("back", getPrePose("back_pose"));
            calib.setLidarFixed("back", fix_back);

            if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_front.bag") != bag_file_lst.end()){
                calib["front"]->push_back("bag_front", pcd_lst[0][0]);
                calib["left"]->push_back("bag_front", pcd_lst[0][1]);
                calib["right"]->push_back("bag_front", pcd_lst[0][2]);
                calib["back"]->push_back("bag_front", pcd_lst[0][3]);
            }
            if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_back.bag") != bag_file_lst.end()){
                calib["front"]->push_back("bag_back", pcd_lst[1][0]);
                calib["left"]->push_back("bag_back", pcd_lst[1][1]);
                calib["right"]->push_back("bag_back", pcd_lst[1][2]);
                calib["back"]->push_back("bag_back", pcd_lst[1][3]);
            }

            if (!fix_front || !fix_left)
                fl_id = calib.addConstraint("front", "bag_front", "left", "bag_front", ml_calib_config.fr_weight);
            if (!fix_front || !fix_right)
                fr_id = calib.addConstraint("front", "bag_front", "right", "bag_front", ml_calib_config.fr_weight);
            if (!fix_left || !fix_right)
                lr_id = calib.addConstraint("left", "bag_front", "right", "bag_front", ml_calib_config.lr_weight);
            if (!fix_back || !fix_left)
                bl_id = calib.addConstraint("left", "bag_back", "back", "bag_back",  ml_calib_config.bl_weight);
            if (!fix_back || !fix_right)
                br_id = calib.addConstraint("right", "bag_back", "back", "bag_back",  ml_calib_config.br_weight);
        }

        auto ros_spin = [&](){
            ros::Rate rate(20);
            while(ros::ok()){

                sensor_msgs::PointCloud2 output_f, output_b;
                if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_front.bag") != bag_file_lst.end()){
                    CloudPtr front_lidar = calib["front"]->getPCDInGlobal("bag_front");
                    CloudPtr left_lidar = calib["left"]->getPCDInGlobal("bag_front");
                    CloudPtr right_lidar = calib["right"]->getPCDInGlobal("bag_front");
                    CloudPtr back_lidar = calib["back"]->getPCDInGlobal("bag_front");

                    pcl::toROSMsg(colorPointCloud(*front_lidar, Eigen::Vector3i(200,200,200)), output_f);
                    output_f.header.stamp = ros::Time::now();
                    output_f.header.frame_id = "lidar";
                    pub_front_bag_f.publish(output_f);

                    pcl::toROSMsg(colorPointCloud(*left_lidar, Eigen::Vector3i(255, 255, 0)), output_f);
                    output_f.header.stamp = ros::Time::now();
                    output_f.header.frame_id = "lidar";
                    pub_front_bag_l.publish(output_f);

                    pcl::toROSMsg(colorPointCloud(*right_lidar, Eigen::Vector3i(255, 0, 255)), output_f);
                    output_f.header.stamp = ros::Time::now();
                    output_f.header.frame_id = "lidar";
                    pub_front_bag_r.publish(output_f);

                    pcl::toROSMsg(colorPointCloud(*back_lidar, Eigen::Vector3i(0,204,0)), output_f);
                    output_f.header.stamp= ros::Time::now();
                    output_f.header.frame_id = "lidar";
                    pub_front_bag_b.publish(output_f);
                }

                if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_back.bag") != bag_file_lst.end()){

                    CloudPtr front_lidar1 = calib["front"]->getPCDInGlobal("bag_back");
                    CloudPtr left_lidar1 = calib["left"]->getPCDInGlobal("bag_back");
                    CloudPtr right_lidar1 = calib["right"]->getPCDInGlobal("bag_back");
                    CloudPtr back_lidar1 = calib["back"]->getPCDInGlobal("bag_back");

                    pcl::toROSMsg(colorPointCloud(*front_lidar1, Eigen::Vector3i(200,200,200)), output_f);
                    output_f.header.stamp = output_b.header.stamp = ros::Time::now();
                    output_f.header.frame_id = output_b.header.frame_id = "lidar";
                    pub_back_bag_f.publish(output_f);
                    
                    pcl::toROSMsg(colorPointCloud(*left_lidar1, Eigen::Vector3i(255, 255, 0)), output_f);
                    output_f.header.stamp = output_b.header.stamp = ros::Time::now();
                    output_f.header.frame_id = output_b.header.frame_id = "lidar";
                    pub_back_bag_l.publish(output_f);

                    pcl::toROSMsg(colorPointCloud(*right_lidar1, Eigen::Vector3i(255, 0, 255)), output_f);
                    output_f.header.stamp = output_b.header.stamp = ros::Time::now();
                    output_f.header.frame_id = output_b.header.frame_id = "lidar";
                    pub_back_bag_r.publish(output_f);

                    pcl::toROSMsg(colorPointCloud(*back_lidar1, Eigen::Vector3i(0,204,0)), output_f);
                    output_f.header.stamp = output_b.header.stamp = ros::Time::now();
                    output_f.header.frame_id = output_b.header.frame_id = "lidar";
                    pub_back_bag_b.publish(output_f);
                }
                ros::spinOnce();
                rate.sleep();
            }
        };
        std::thread thd_ros_spin(ros_spin);
        thd_ros_spin.detach();
        bool pub_ground = false;
        while(ros::ok()){
            std::unique_lock<std::mutex> lock(mtx_config);
            if (config_changed){
                updatePatchWorkParameters();
                calib["front"]->updateGround(ml_calib_config.front_lidar_ground_radius);
                calib["left"]->updateGround(ml_calib_config.left_lidar_ground_radius);
                calib["right"]->updateGround(ml_calib_config.right_lidar_ground_radius);
                calib["back"]->updateGround(ml_calib_config.back_lidar_ground_radius);
                calib.updatePlaneGroundNoise(ml_calib_config.ground_noise, ml_calib_config.plane_noise);

                if(fl_id >= 0) calib.updateWeight(fl_id, ml_calib_config.fl_weight);
                if(fr_id >= 0) calib.updateWeight(fr_id, ml_calib_config.fr_weight);
                if(lr_id >= 0) calib.updateWeight(lr_id, ml_calib_config.lr_weight);
                if(bl_id >= 0) calib.updateWeight(bl_id, ml_calib_config.bl_weight);
                if(br_id >= 0) calib.updateWeight(br_id, ml_calib_config.br_weight);

                calib.threshold_plane = ml_calib_config.plane_threshold;
                Eigen::Vector3d map_correct_trans = Eigen::Vector3d(ml_calib_config.x, ml_calib_config.y, ml_calib_config.z);
                Eigen::Vector3d map_correct_rot = Eigen::Vector3d(ml_calib_config.pitch_x, ml_calib_config.roll_y, ml_calib_config.yaw_z);
                calib.correctMapPrePose(map_correct_rot, map_correct_trans);
                pub_ground = true;
                config_changed = false;
            }else{
                pub_ground = false;
            }
            lock.unlock();

            std::cout << "\n========================================================================================" << std::endl;
            ros::Time start = ros::Time::now();
            calib.step(ml_calib_config.use_ground_optimization);
            std::cout << "Opt cost time: " << (ros::Time::now() - start).toSec() << std::endl;

            std::cout << "\n############################ Front transformation ###########################\n";
            std::cout << "Euler rpy(rad), xyz(m): \n" << calib["front"]->getPose().topLeftCorner<3,3>().eulerAngles(0, 1, 2).transpose().format(eigen_fmt) 
                        << ", " << calib["front"]->getPose().topRightCorner(3,1).transpose().format(eigen_fmt) << std::endl;
            std::cout << "Matrix: \n" << calib["front"]->getPose().format(eigen_fmt) << std::endl;
            std::cout << "\n############################ Left transformation ############################\n";
            std::cout << "Euler rpy(rad), xyz(m): \n" << calib["left"]->getPose().topLeftCorner<3,3>().eulerAngles(0, 1, 2).transpose().format(eigen_fmt) 
                        << ", " << calib["left"]->getPose().topRightCorner(3,1).transpose().format(eigen_fmt) << std::endl;
            std::cout << "Matrix: \n" << calib["left"]->getPose().format(eigen_fmt) << std::endl;
            std::cout << "\n############################ Right transformation ###########################\n";
            std::cout << "Euler rpy(rad), xyz(m): \n" << calib["right"]->getPose().topLeftCorner<3,3>().eulerAngles(0, 1, 2).transpose().format(eigen_fmt)
                        << ", " << calib["right"]->getPose().topRightCorner(3,1).transpose().format(eigen_fmt) << std::endl;
            std::cout << "Matrix: \n" << calib["right"]->getPose().format(eigen_fmt) << std::endl;
            std::cout << "\n############################ Back transformation ############################\n";
            std::cout << "Euler rpy(rad), xyz(m): \n" << calib["back"]->getPose().topLeftCorner<3,3>().eulerAngles(0, 1, 2).transpose().format(eigen_fmt)
                        << ", " << calib["back"]->getPose().topRightCorner(3,1).transpose().format(eigen_fmt) << std::endl;
            std::cout << "Matrix: \n" << calib["back"]->getPose().format(eigen_fmt) << std::endl;

            visualization_msgs::MarkerArray marker_array_front, marker_array_back;
            visualization_msgs::MarkerArray marker_array_norm_front, marker_array_norm_back;

            pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> intersected_front_pcd(new pcl::PointCloud<pcl::PointXYZRGBA>());
            intersected_front_pcd->header.frame_id = "lidar";
            intersected_front_pcd->header.stamp = ros::Time::now().toNSec() / 1000;

            pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> intersected_back_pcd(new pcl::PointCloud<pcl::PointXYZRGBA>());
            intersected_front_pcd->header.frame_id = "lidar";
            intersected_front_pcd->header.stamp = ros::Time::now().toNSec() / 1000;

            sensor_msgs::PointCloud2 ground_msg;
            CloudPtr ground_pcd = pcl::make_shared<PointCloud>();
            auto add_pcd_by_indices = [](CloudPtr& source, CloudPtr& target, std::set<int>& indices){
                for (int idx : indices)
                    source->push_back(target->at(idx));
            };

            if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_front.bag") != bag_file_lst.end()){
                CloudPtr front_lidar = calib["front"]->getPCDInGlobal("bag_front");
                CloudPtr left_lidar = calib["left"]->getPCDInGlobal("bag_front");
                CloudPtr right_lidar = calib["right"]->getPCDInGlobal("bag_front");
                CloudPtr back_lidar = calib["back"]->getPCDInGlobal("bag_front");
                
                if (pub_ground){
                    ground_pcd->clear();
                    add_pcd_by_indices(ground_pcd, front_lidar, calib["front"]->ground_indices["bag_front"]);
                    add_pcd_by_indices(ground_pcd, left_lidar, calib["left"]->ground_indices["bag_front"]);
                    add_pcd_by_indices(ground_pcd, right_lidar, calib["right"]->ground_indices["bag_front"]);
                    add_pcd_by_indices(ground_pcd, back_lidar, calib["back"]->ground_indices["bag_front"]);
                    pcl::toROSMsg(*ground_pcd, ground_msg);
                    ground_msg.header.frame_id = "lidar";
                    ground_msg.header.stamp = ros::Time::now();
                    pub_groud_front.publish(ground_msg);
                }

                if (fl_id >= 0){
                    auto detail_fl = calib.getConstraintDetails(fl_id);
                    constructMarker(marker_array_front, detail_fl, front_lidar, left_lidar, intersected_front_pcd, 0);
                    constructMarker(marker_array_norm_front, detail_fl, front_lidar, left_lidar, intersected_front_pcd, 1);
                }
                if (fr_id >= 0){
                    auto detail_fr = calib.getConstraintDetails(fr_id);
                    constructMarker(marker_array_front, detail_fr, front_lidar, right_lidar, intersected_front_pcd, 0);
                    constructMarker(marker_array_norm_front, detail_fr, front_lidar, right_lidar, intersected_front_pcd, 1);
                }
                if (lr_id >= 0){
                    auto detail_lr = calib.getConstraintDetails(lr_id);
                    constructMarker(marker_array_front, detail_lr, left_lidar, right_lidar, intersected_front_pcd, 0);
                    constructMarker(marker_array_norm_front, detail_lr, left_lidar, right_lidar, intersected_front_pcd, 1);
                }
            }
            if (std::find(bag_file_lst.begin(), bag_file_lst.end(), "lidar_back.bag") != bag_file_lst.end()){
                CloudPtr front_lidar1 = calib["front"]->getPCDInGlobal("bag_back");
                CloudPtr left_lidar1 = calib["left"]->getPCDInGlobal("bag_back");
                CloudPtr right_lidar1 = calib["right"]->getPCDInGlobal("bag_back");
                CloudPtr back_lidar1 = calib["back"]->getPCDInGlobal("bag_back");
                
                if (pub_ground){
                    ground_pcd->clear();
                    add_pcd_by_indices(ground_pcd, front_lidar1, calib["front"]->ground_indices["bag_back"]);
                    add_pcd_by_indices(ground_pcd, left_lidar1, calib["left"]->ground_indices["bag_back"]);
                    add_pcd_by_indices(ground_pcd, right_lidar1, calib["right"]->ground_indices["bag_back"]);
                    add_pcd_by_indices(ground_pcd, back_lidar1, calib["back"]->ground_indices["bag_back"]);
                    pcl::toROSMsg(*ground_pcd, ground_msg);
                    ground_msg.header.frame_id = "lidar";
                    ground_msg.header.stamp = ros::Time::now();
                    pub_groud_back.publish(ground_msg);
                }

                if (bl_id >= 0){
                    auto detail_bl = calib.getConstraintDetails(bl_id);
                    constructMarker(marker_array_back, detail_bl, left_lidar1, back_lidar1, intersected_back_pcd, 0);
                    constructMarker(marker_array_norm_back, detail_bl, left_lidar1, back_lidar1, intersected_back_pcd, 1);
                }
                if (br_id >= 0){
                    auto detail_br = calib.getConstraintDetails(br_id);
                    constructMarker(marker_array_back, detail_br, right_lidar1, back_lidar1, intersected_back_pcd, 0);
                    constructMarker(marker_array_norm_back, detail_br, right_lidar1, back_lidar1, intersected_back_pcd, 1);
                }
            }

            pub_constraint_front.publish(marker_array_front);
            pub_constraint_back.publish(marker_array_back);
            pub_normal_front.publish(marker_array_norm_front);
            pub_normal_back.publish(marker_array_norm_back);

            sensor_msgs::PointCloud2 output_front_intersected;
            pcl::toROSMsg(*intersected_front_pcd, output_front_intersected);
            output_front_intersected.header.frame_id = "lidar";
            output_front_intersected.header.stamp = ros::Time::now();
            pub_intersection_front.publish(output_front_intersected);

            sensor_msgs::PointCloud2 output_back_intersected;
            pcl::toROSMsg(*intersected_back_pcd, output_back_intersected);
            output_back_intersected.header.frame_id = "lidar";
            output_back_intersected.header.stamp = ros::Time::now();
            pub_intersection_back.publish(output_back_intersected);
        }

    }else{

    }


   
    return 0;
}
