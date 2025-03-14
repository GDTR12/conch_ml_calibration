#include "ros/ros.h"
#include "bag2pcd/bag2pcd.hpp"
#include <yaml-cpp/yaml.h>
#include "multi_lidar_calibration/ml_calib.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include "multi_lidar_calibration/ground_factor.hpp"
#include "multi_lidar_calibration/joint_optimization.hpp"
#include "patchwork/patchworkpp.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>


using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using CloudPtr = pcl::shared_ptr<PointCloud>;
using ConstCloudPtr = pcl::shared_ptr<const PointCloud>;
using OctreeT = pcl::octree::OctreePointCloud<PointT>;
namespace fs = std::filesystem;

template<typename PointT>
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
}

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

pcl::shared_ptr<pcl::PointCloud<PointT>> pcdDistanceRejectPoint(pcl::shared_ptr<pcl::PointCloud<PointT>> pcd, float begin_distance, float end_distance, std::vector<int>& indices)
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
}

Eigen::Affine3d groundOptimization(CloudPtr pcd)
{
    patchwork::Params patchwork_parameters;
    patchwork_parameters.verbose = true;
    patchwork::PatchWorkpp patchworkpp(patchwork_parameters);
   
    Eigen::MatrixXf cloud;
    cloud.resize(pcd->size(), 4);
    for (int i = 0; i < pcd->size(); i++)
    {
        cloud.row(i) << pcd->at(i).x, pcd->at(i).y, pcd->at(i).z, pcd->at(i).intensity;
    }
    patchworkpp.estimateGround(cloud);
    Eigen::VectorXi ground_idx    = patchworkpp.getGroundIndices();
    std::vector<int> std_vec(ground_idx.data(), ground_idx.data() + ground_idx.size());
    pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/ground.pcd", *pcd, std_vec);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Symbol key('p', 0);

    for (auto idx: std_vec)
    {
        gtsam::SharedNoiseModel noise_model;
        // if (pcd->at(idx).z > 1.5){
        //     noise_model = gtsam::noiseModel::mEstimator::Tukey::Create(4.685);
        // }else{
            noise_model = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                gtsam::noiseModel::Isotropic::Sigma(1, 0.1)
            );
        // }
        graph.emplace_shared<ml_calib::LiDARGroundPointFactor>(
            key, 0, (Eigen::Vector3d() << pcd->at(idx).getVector3fMap().cast<double>()).finished(), noise_model);
    }
    
    gtsam::Values initial;
    initial.insert(key, gtsam::Vector3(0,0,0));

    gtsam::LevenbergMarquardtParams params;
    // params.setVerbosity("ERROR");  // 输出优化信息
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values result_ = optimizer.optimize();

    Eigen::Vector3d result = result_.at<gtsam::Vector3>(key);
    std::cout << "Opt result: " << result.transpose() << std::endl;
    
    Eigen::Affine3d trans_mat = Eigen::Affine3d::Identity();
    trans_mat.translate(Eigen::Vector3d(0, 0, result.z()));
    trans_mat.rotate(Eigen::AngleAxisd(result.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(result.x(), Eigen::Vector3d::UnitX()));
    return trans_mat;
}


void searchInterSection(ConstCloudPtr pcd0, 
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


void visualizeIntersection(pcl::IndicedMerge<PointT>& merged_back, pcl::KdTreeFLANN<PointT>& kd_merged_back, std::vector<std::pair<int, int>>& idx_pairs)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    for (auto [id0, id1] : idx_pairs)
    {
        int id0_ = merged_back.idxLocal2Global("back", id0);
        int id1_ = merged_back.idxLocal2Global("left", id1);
        std::cout << id0 << " " << id1 << std::endl;
        pcl::Indices indices;
        std::vector<float> dists;
        kd_merged_back.radiusSearch(merged_back.at(id0_), 0.5, indices, dists);

        Eigen::Vector3f normal;
        if (0.0 < checkPlane<PointT>(merged_back.merged_cloud, indices, normal)){
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
}
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


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ml_calib");
    ros::NodeHandle nh("~");
    std::string project_path, bag_folder, launch_cmd_dir, config_url, g3reg_config_path;
    nh.getParam("project_path", project_path);
    nh.getParam("bag_folder_path", bag_folder);
    nh.getParam("launch_command_dir", launch_cmd_dir);

    std::string topic_pub_front("/ml_calib/front_lidar"), topic_pub_left("/ml_calib/left_lidar"), topic_pub_right("/ml_calib/right_lidar"), topic_pub_back("/ml_calib/back_lidar");
    ros::Publisher pub_front = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_front, 10);
    ros::Publisher pub_left = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_left, 10);
    ros::Publisher pub_right = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_right, 10);
    ros::Publisher pub_back = nh.advertise<sensor_msgs::PointCloud2>(topic_pub_back, 10);

    g3reg_config_path = fs::path(project_path) / "third_party/G3Reg/configs";
    config_url = fs::path(project_path) / "config/multi_scene_calib.yaml";

    YAML::Node root = YAML::LoadFile(config_url);

    std::vector<std::string> lidar_topics = root["lidar_topics"].as<std::vector<std::string>>();

    std::vector<std::string> bag_file_lst = root["bag_files"].as<std::vector<std::string>>();
    fs::path bag_path = fs::path(launch_cmd_dir) / fs::path(bag_folder);

    std::vector<std::vector<CloudPtr>> pcd_lst;

    for(const auto bag_file: bag_file_lst)
    {
        fs::path bag_url = bag_path / bag_file;
        std::vector<CloudPtr>& bag_clouds = pcd_lst.emplace_back();
        if (fs::exists(bag_url)){
            for (const std::string& topic : lidar_topics)
            {
                std::cout << "Try open bagfile: " << bag_url << std::endl;        
                std::vector<CloudPtr> pcd;
                bag2pcd::rosbag2PointCloud<PointT>(bag_url, topic, pcd);
                bag_clouds.push_back(pcd[0]);
            }
        }else{
            std::cout << "File doesn't exist: " << bag_url << std::endl;        
            exit(0);
        }
    }


    pcl::G3Reg<PointT, PointT> reg;
    reg.setG3RegParams(fs::path(g3reg_config_path) / "apollo_lc_bm/fpfh_pagor.yaml");

    CloudPtr front_lidar = pcd_lst[0][0];
    CloudPtr left_lidar = pcd_lst[0][1];
    CloudPtr right_lidar = pcd_lst[0][2];

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*front_lidar, *front_lidar, indices);
    pcl::removeNaNFromPointCloud(*left_lidar, *left_lidar, indices);
    pcl::removeNaNFromPointCloud(*right_lidar, *right_lidar, indices);

    indices.clear();
    CloudPtr sectored_left_pcd = pcdSector<PointT>(left_lidar, -180, 0, indices);
    indices.clear();
    CloudPtr sectored_right_pcd = pcdSector<PointT>(right_lidar, 0, 180, indices);

    reg.setInputCloud(sectored_left_pcd);
    reg.setInputTarget(front_lidar);
    reg.align(*sectored_left_pcd);
    Eigen::Matrix4f trans_mat_left = reg.getFinalTransformation();

    reg.setInputCloud(sectored_right_pcd);
    reg.setInputTarget(front_lidar);
    reg.align(*sectored_right_pcd);
    Eigen::Matrix4f trans_mat_right = reg.getFinalTransformation();

    pcl::transformPointCloud(*left_lidar, *left_lidar, trans_mat_left);
    pcl::transformPointCloud(*right_lidar, *right_lidar, trans_mat_right);

    // PointCloud aa;
    // aa += *right_lidar;
    // aa += *left_lidar;
    // aa += *front_lidar;
    // std::cout << aa.size() << std::endl;
    // pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/prev_merge.pcd", aa);
    CloudPtr front_merge = pcl::make_shared<PointCloud>();
    CloudPtr back_merge = pcl::make_shared<PointCloud>();

    Eigen::Affine3d tans_front = groundOptimization(front_lidar);
    Eigen::Affine3d tans_left = groundOptimization(left_lidar);
    Eigen::Affine3d tans_right = groundOptimization(right_lidar);

    pcl::transformPointCloud(*right_lidar, *right_lidar, tans_right);
    pcl::transformPointCloud(*front_lidar, *front_lidar, tans_front);
    pcl::transformPointCloud(*left_lidar, *left_lidar, tans_left);

    CloudPtr front_lidar1 = pcd_lst[1][0];
    CloudPtr left_lidar1 = pcd_lst[1][1];
    CloudPtr right_lidar1 = pcd_lst[1][2];
    CloudPtr back_lidar1 = pcd_lst[1][3];

    pcl::transformPointCloud(*front_lidar1, *front_lidar1, tans_front);
    pcl::transformPointCloud(*left_lidar1, *left_lidar1, tans_left * Eigen::Affine3d(trans_mat_left.cast<double>()));
    pcl::transformPointCloud(*right_lidar1, *right_lidar1, tans_right * Eigen::Affine3d(trans_mat_right.cast<double>()));


    pcl::IndicedMerge<PointT> merged_back;
    merged_back.push_back("front", front_lidar1);
    merged_back.push_back("left", left_lidar1);
    merged_back.push_back("right", right_lidar1);
    

    CloudPtr result_pcd = pcl::make_shared<PointCloud>();
    reg.setInputCloud(back_lidar1);
    reg.setInputTarget(merged_back.merged_cloud);
    reg.align(*result_pcd);
    Eigen::Matrix4f trans_back = reg.getFinalTransformation();
    pcl::transformPointCloud(*back_lidar1, *back_lidar1, trans_back);

    Eigen::Affine3d trans_back_ground = groundOptimization(back_lidar1);

    pcl::transformPointCloud(*back_lidar1, *back_lidar1, trans_back_ground);

    merged_back.push_back("back", back_lidar1);


    auto opt_intersection = [&](){

        gtsam::NonlinearFactorGraph graph;
        gtsam::Symbol key_pose_front('p', 0);
        gtsam::Symbol key_pose_left('p', 1);
        gtsam::Symbol key_pose_right('p', 2);
        gtsam::Symbol key_pose_back('p', 3);


        gtsam::SharedNoiseModel noise_model;
        noise_model = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Isotropic::Sigma(1, 0.1));

        float voxel_size = 1;
        OctreeT back_octree(voxel_size);
        back_octree.setInputCloud(back_lidar1);
        back_octree.addPointsFromInputCloud();
        std::vector<std::pair<int, int>> idx_pairs;

        pcl::KdTreeFLANN<PointT> kd_left1;
        kd_left1.setInputCloud(left_lidar1);
        searchInterSection(back_lidar1, back_octree, left_lidar1, kd_left1, voxel_size, idx_pairs);
        // std::cout << "Intersection: " << idx_pairs.size() << std::endl;
        for (const auto& [idx0, idx1] : idx_pairs)
        {
            graph.emplace_shared<ml_calib::LidarJointOptFactor>(key_pose_back, key_pose_left, noise_model,
                                                                back_lidar1->at(idx0).getVector3fMap().cast<double>(),
                                                                left_lidar1->at(idx1).getVector3fMap().cast<double>(),
                                                                Eigen::Vector3d::Zero(),
                                                                false);
            
        }

        idx_pairs.clear();
        pcl::KdTreeFLANN<PointT> kd_right1;
        kd_right1.setInputCloud(right_lidar1);
        searchInterSection(back_lidar1, back_octree, right_lidar1, kd_right1, voxel_size, idx_pairs);
        for (const auto& [idx0, idx1] : idx_pairs)
        {
            graph.emplace_shared<ml_calib::LidarJointOptFactor>(key_pose_back, key_pose_right, noise_model,
                                                                back_lidar1->at(idx0).getVector3fMap().cast<double>(),
                                                                right_lidar1->at(idx1).getVector3fMap().cast<double>(),
                                                                Eigen::Vector3d::Zero(),
                                                                false);
            
        }

        OctreeT front_octree(voxel_size);
        front_octree.setInputCloud(front_lidar);
        front_octree.addPointsFromInputCloud();

        idx_pairs.clear();
        pcl::KdTreeFLANN<PointT>  kd_left0;
        kd_left0.setInputCloud(left_lidar);
        searchInterSection(front_lidar, front_octree, left_lidar, kd_left0, voxel_size, idx_pairs);
        for (const auto& [idx0, idx1] : idx_pairs)
        {
            graph.emplace_shared<ml_calib::LidarJointOptFactor>(key_pose_front, key_pose_left, noise_model,
                                                                front_lidar->at(idx0).getVector3fMap().cast<double>(),
                                                                left_lidar->at(idx1).getVector3fMap().cast<double>(),
                                                                Eigen::Vector3d::Zero(),
                                                                false);
            
        }

        idx_pairs.clear();
        pcl::KdTreeFLANN<PointT> kd_right0;
        kd_right0.setInputCloud(right_lidar);
        searchInterSection(front_lidar, front_octree, right_lidar, kd_right0, voxel_size, idx_pairs);
        for (const auto& [idx0, idx1] : idx_pairs)
        {
            graph.emplace_shared<ml_calib::LidarJointOptFactor>(key_pose_front, key_pose_right, noise_model,
                                                                front_lidar->at(idx0).getVector3fMap().cast<double>(),
                                                                right_lidar->at(idx1).getVector3fMap().cast<double>(),
                                                                Eigen::Vector3d::Zero(),
                                                                false);
            
        }

        OctreeT left_octree(voxel_size);
        left_octree.setInputCloud(left_lidar);
        left_octree.addPointsFromInputCloud();

        idx_pairs.clear();
        searchInterSection(left_lidar, left_octree, right_lidar, kd_right0, voxel_size, idx_pairs);
        for (const auto& [idx0, idx1] : idx_pairs)
        {
            graph.emplace_shared<ml_calib::LidarJointOptFactor>(key_pose_left, key_pose_right, noise_model,
                                                                left_lidar->at(idx0).getVector3fMap().cast<double>(),
                                                                right_lidar->at(idx1).getVector3fMap().cast<double>(),
                                                                Eigen::Vector3d::Zero(),
                                                                false);
            
        }   
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(key_pose_front, gtsam::Pose3::Identity(), gtsam::noiseModel::Constrained::All(6)));

        gtsam::Values initial;
        initial.insert(key_pose_front, gtsam::Pose3::Identity());
        initial.insert(key_pose_left, gtsam::Pose3::Identity());
        initial.insert(key_pose_right, gtsam::Pose3::Identity());
        initial.insert(key_pose_back, gtsam::Pose3::Identity());

        gtsam::LevenbergMarquardtParams params;
        // params.setVerbosity("ERROR");  // 输出优化信息
        params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
        gtsam::Values result_ = optimizer.optimize();
        
        gtsam::Pose3 tr_f = result_.at<gtsam::Pose3>(key_pose_front);
        gtsam::Pose3 tr_l = result_.at<gtsam::Pose3>(key_pose_left);
        gtsam::Pose3 tr_r = result_.at<gtsam::Pose3>(key_pose_right);
        gtsam::Pose3 tr_b = result_.at<gtsam::Pose3>(key_pose_back);

        pcl::transformPointCloud(*front_lidar, *front_lidar, tr_f.matrix());
        pcl::transformPointCloud(*front_lidar1, *front_lidar1, tr_f.matrix());

        pcl::transformPointCloud(*left_lidar, *left_lidar, tr_l.matrix());
        pcl::transformPointCloud(*left_lidar1, *left_lidar1, tr_l.matrix());

        pcl::transformPointCloud(*right_lidar, *right_lidar, tr_r.matrix());
        pcl::transformPointCloud(*right_lidar1, *right_lidar1, tr_r.matrix());

        pcl::transformPointCloud(*back_lidar1, *back_lidar1, tr_b.matrix());
    };

    ros::Rate rate(10); // 发布频率 10Hz
    while(ros::ok())
    {
        opt_intersection();
        sensor_msgs::PointCloud2 output_f;
        pcl::toROSMsg(colorPointCloud(*front_lidar1, Eigen::Vector3i(200,200,200)), output_f);
        output_f.header.stamp = ros::Time::now();
        output_f.header.frame_id = "lidar";
        pub_front.publish(output_f);

        sensor_msgs::PointCloud2 output_l;
        pcl::toROSMsg(colorPointCloud(*left_lidar1, Eigen::Vector3i(0,255,0)), output_l);
        output_l.header.stamp = ros::Time::now();
        output_l.header.frame_id = "lidar";
        pub_left.publish(output_l);

        sensor_msgs::PointCloud2 output_r;
        pcl::toROSMsg(colorPointCloud(*right_lidar1, Eigen::Vector3i(255,0,255)), output_r);
        output_r.header.stamp = ros::Time::now();
        output_r.header.frame_id = "lidar";
        pub_right.publish(output_r);

        sensor_msgs::PointCloud2 output_b;
        pcl::toROSMsg(colorPointCloud(*back_lidar1, Eigen::Vector3i(255,255,0)), output_b);
        output_b.header.stamp = ros::Time::now();
        output_b.header.frame_id = "lidar";
        pub_back.publish(output_b);

        rate.sleep();
    }

    // // Eigen::Matrix4f trans_mat = reg.getFinalTransformation();
    // ml_calib::MLCalib calib;
    // calib.push_back(front_lidar);
    // calib.push_back(left_lidar);
    // calib.push_back(right_lidar);
    // calib.push_back(back_lidar);
    // calib.run();
    return 0;
}
