#include "multi_lidar_calibration/ml_calib.hpp"
#include "bag2pcd/bag2pcd.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <memory>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "multi_lidar_calibration/ground_factor.hpp"
#include "multi_lidar_calibration/ground_joint_opt.hpp"
#include "multi_lidar_calibration/joint_optimization.hpp"
#include <stdexcept>

namespace ml_calib
{

void MLiDAR::push_back(const std::string& name, CloudPtr cloud)
{
    CloudPtr source_pcd = pcl::make_shared<PointCloud>();
    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud, *source_pcd, indices);
    source_pcds[name] = source_pcd;
    
    patchwork::PatchWorkpp patchworkpp(patchwork_parameters);
    Eigen::MatrixXf patch_cloud;
    patch_cloud.resize(source_pcd->size(), 4);
    for (int i = 0; i < source_pcd->size(); i++)
    {
        patch_cloud.row(i) << source_pcd->at(i).x, source_pcd->at(i).y, source_pcd->at(i).z, source_pcd->at(i).intensity;
    }
    patchworkpp.estimateGround(patch_cloud);
    Eigen::VectorXi ground_idx    = patchworkpp.getGroundIndices();
    ground_indices[name] = std::set<int>();
    for (int i = 0; i < ground_idx.size(); i++)
    {
        ground_indices[name].insert(ground_idx[i]);
    }
}

void MLiDAR::addTargetLeaf(const std::string& source_name, const std::shared_ptr<MLiDAR>& target_lidar, const std::string& target_name)
{
    if (source_pcds.find(source_name) == source_pcds.end()){
        throw std::runtime_error("Source point cloud not found: " + source_name);
    }
    if (target_lidar->source_pcds.find(target_name) == target_lidar->source_pcds.end()){
        throw std::runtime_error("Target point cloud not found: " + target_name);
    }

    MLiDARLeaf leaf;
    leaf.source_name = source_name;
    leaf.target_lidar = target_lidar;
    leaf.target_name = target_name;
    target_leaves.push_back(leaf);
}

CloudPtr MLiDAR::getPCDInGlobal(const std::string& pcd_name)
{
    if (source_pcds.find(pcd_name) != source_pcds.end()){
        CloudPtr pcd = source_pcds[pcd_name];
        CloudPtr transformed_pcd = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*pcd, *transformed_pcd, getPose().cast<float>());
        return transformed_pcd;
    }
    throw std::runtime_error("Source point cloud not found: " + pcd_name);
}

void MLiDAR::updateGround(double ground_radius)
{
    patchwork::PatchWorkpp patchworkpp(patchwork_parameters);
    ground_indices.clear();
    for(const auto& [name, source_pcd] : source_pcds)
    {
        Eigen::MatrixXf patch_cloud;
        patch_cloud.resize(source_pcd->size(), 4);
        for (int i = 0; i < source_pcd->size(); i++)
        {
            patch_cloud.row(i) << source_pcd->at(i).x, source_pcd->at(i).y, source_pcd->at(i).z, source_pcd->at(i).intensity;
        }
        patchworkpp.estimateGround(patch_cloud);
        Eigen::VectorXi ground_idx    = patchworkpp.getGroundIndices();
        ground_indices[name] = std::set<int>();
        for (int i = 0; i < ground_idx.size(); i++)
        {
            if (source_pcd->at(ground_idx[i]).getVector3fMap().norm() < ground_radius){
                ground_indices[name].insert(ground_idx[i]);
            }
        }
    }
}

void MLiDAR::correctPose(const Eigen::Matrix4d& mat)
{
    if (mat.determinant() < 1e-5){
        throw std::runtime_error("correct matrix is not invertable!");
    }
    correct_mat = mat;
}

void MLiDAR::correctPose(const Eigen::Vector3d& euler, const Eigen::Vector3d& translation)
{
    auto bound_to_mat = [](const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) -> Eigen::Affine3d {
        Eigen::Affine3d mat(Eigen::Affine3d::Identity());
        mat.translation() = xyz;
        mat.rotate(Eigen::AngleAxisd(DEG2RAD(rpy.x()), Eigen::Vector3d::UnitX()));
        mat.rotate(Eigen::AngleAxisd(DEG2RAD(rpy.y()), Eigen::Vector3d::UnitY()));
        mat.rotate(Eigen::AngleAxisd(DEG2RAD(rpy.z()), Eigen::Vector3d::UnitZ()));
        return mat;
    };
    Eigen::Matrix4d fix_f = bound_to_mat(translation, euler).matrix();
    correctPose(fix_f);
}

CloudPtr MLiDAR::operator[](const std::string& name)
{
    if (source_pcds.find(name) != source_pcds.end()){
        return source_pcds[name];
    }
    throw std::runtime_error("Source point cloud not found: " + name);
}




MLCalib::MLCalib(const std::string &g3reg_config_path)
{
    g3reg_config_path_ = g3reg_config_path;
    reg.setG3RegParams(g3reg_config_path_);
}

MLCalib::~MLCalib(){}


void MLCalib::addLiDAR(const std::string& name)
{
    if (lidars.find(name) != lidars.end()){
        throw std::runtime_error("Lidar already exists: " + name);
    }
    std::shared_ptr<MLiDAR> lidar = std::make_shared<MLiDAR>(name, lidars.size());
    lidars[name] = lidar;
}


std::shared_ptr<MLiDAR>& MLCalib::operator[](const std::string& name)
{
    if (lidars[name] != nullptr){
        return lidars[name];
    }
    throw std::runtime_error("Lidar not found: " + name);
}


void MLCalib::setRootLidar(const std::string& lidar_name)
{
    if (lidars.find(lidar_name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + lidar_name);
    }
    root_lidar_name_ = lidar_name;
}


void MLCalib::setRootLidarHeight(double height)
{
    root_lidar_height = height;
    groundOpt(root_lidar_name_, lidars[root_lidar_name_]->source_pcds.begin()->first);
}


void MLCalib::setLidarPose(const std::string& name, const Eigen::Matrix4d& pose)
{
    if (lidars.find(name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + name);
    }
    lidars[name]->pose = pose;
}


void MLCalib::setLidarFixed(const std::string& name, bool fixed)
{
    if (lidars.find(name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + name);
    }
    if (fixed){
        if (std::find(fixed_lidars.begin(), fixed_lidars.end(), name) == fixed_lidars.end()){
            fixed_lidars.push_back(name);
        }
    }else{
        auto it = std::find(fixed_lidars.begin(), fixed_lidars.end(), name);
        if (it != fixed_lidars.end()){
            fixed_lidars.erase(it);
        }
    }
    lidars[name]->initialized = true;
}


int MLCalib::addConstraint(const std::string& lidar0_name, const std::string& pcd0_name,
                    const std::string& lidar1_name, const std::string& pcd1_name, double weight)
{
    if (lidars.find(lidar0_name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + lidar0_name);
    }
    if (lidars[lidar0_name]->source_pcds.find(pcd0_name) == lidars[lidar0_name]->source_pcds.end()){
        throw std::runtime_error("Source point cloud not found: " + pcd0_name);
    }

    if (lidars.find(lidar1_name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + lidar1_name);
    }
    if (lidars[lidar1_name]->source_pcds.find(pcd1_name) == lidars[lidar1_name]->source_pcds.end()){
        throw std::runtime_error("Source point cloud not found: " + pcd1_name);
    }
    constraint_infos.push_back({lidar0_name, pcd0_name, lidar1_name, pcd1_name, weight});
    return constraint_infos.size() - 1;
}


void MLCalib::updateWeight(const int id, double weight)
{
    if (id < 0 || id >= constraint_infos.size()){
        throw std::runtime_error("Constraint ID out of range: " + std::to_string(id));
    }
    constraint_infos[id].weight = weight;
}



void MLCalib::buildGroundConstraints()
{
    gtsam::NonlinearFactorGraph graph;
    constraint_details.clear();
    constraint_details.resize(constraint_infos.size());
    for (int i = 0; i < constraint_infos.size(); i++)
    {
        auto& info = constraint_infos[i];
        auto lidar0 = lidars[info.lidar0_name];
        auto pcd0 = lidar0->getPCDInGlobal(info.pcd0_name);
        auto lidar1 = lidars[info.lidar1_name];
        auto pcd1 = lidar1->getPCDInGlobal(info.pcd1_name);
        
        pcl::IndicedMerge<PointT> merged;
        merged.push_back(info.lidar0_name, pcd0);
        merged.push_back(info.lidar1_name, pcd1);
        
        pcl::KdTreeFLANN<PointT> kd_merged;
        kd_merged.setInputCloud(merged.merged_cloud);
        
        std::unordered_map<int, int> idx_pairs;

        searchInterSection(pcd0, pcd1, find_intersection_threshold, idx_pairs);
        
        for (const auto& [id0_, id1_] : idx_pairs)
        {
            int id0 = merged.idxLocal2Global(info.lidar0_name, id0_);
            int id1 = merged.idxLocal2Global(info.lidar1_name, id1_);


            ConstCloudPtr merged_cloud = merged.merged_cloud;
            pcl::Indices indices;
            std::vector<float> dists;
            kd_merged.radiusSearch(merged_cloud->at(id0), plane_serach_radius, indices, dists);
            Eigen::Vector3f normal;
            double plane_score = checkPlane<PointT>(merged_cloud, indices, normal);
            if (threshold_plane < plane_score){
                if (lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() &&
                        lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end()){
                    normal = Eigen::Vector3f::UnitZ();
                    normal.normalize();
                }

                double distance = normal.dot(pcd0->at(id0_).getVector3fMap() - pcd1->at(id1_).getVector3fMap());
                if (distance < project_distance_threshold){

                    if ((lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() &&
                        lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end()) || 
                        pcd0->at(id0_).getVector3fMap().norm() < 5){
                        graph.emplace_shared<ml_calib::GroundJointOptFactor>(pcd0->at(id0_).getVector3fMap().cast<double>(),
                                                                            pcd1->at(id1_).getVector3fMap().cast<double>(),
                                                                            lidar0->symbol, lidar1->symbol, ground_noise_model, 
                                                                            info.weight);
                        auto& detal = constraint_details[i].emplace_back();
                            detal.id0 = id0_;
                            detal.id1 = id1_;
                            detal.distance = std::abs(distance);
                            detal.normal = normal.cast<double>();
                            detal.label = "ground";
                        }
                    
                }
            }
        }
    }
    for (const auto& fixed_lidar : fixed_lidars)
    {
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(lidars[fixed_lidar]->symbol, gtsam::Vector3::Zero(), gtsam::noiseModel::Constrained::All(3)));
    }

    gtsam::Values initial;
    for (const auto& [lidar_name, lidar] : lidars)
    {
        if (lidar->initialized){
            initial.insert(lidar->symbol, gtsam::Vector3(0,0,0));
        }
    }
    gtsam::LevenbergMarquardtParams params;
    // params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    double init_error = optimizer.error();
    std::cout << "Initilization cost: " << init_error << std::endl;
    gtsam::Values result_ = optimizer.optimize();
    double opted_error = optimizer.error();
    std::cout << "Optimized cost: " << opted_error << std::endl;
    std::cout << "Cost Change: " << init_error - opted_error << std::endl;
    
    for (const auto& [lidar_name, lidar] : lidars)
    {
        if (lidar->initialized){
            auto result = result_.at<gtsam::Vector3>(lidar->symbol);
            Eigen::Affine3d t_f = Eigen::Affine3d::Identity();
            t_f.translate(Eigen::Vector3d(0, 0, result.z()));
            t_f.rotate(Eigen::AngleAxisd(result.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(result.x(), Eigen::Vector3d::UnitX()));
            lidar->pose = lidar->correct_mat.inverse() * t_f.matrix() * lidar->getPose();
        }
    }
}

void MLCalib::buildConstraints()
{
    gtsam::NonlinearFactorGraph graph;
    constraint_details.clear();
    constraint_details.resize(constraint_infos.size());
    for (int i = 0; i < constraint_infos.size(); i++)
    {
        auto& info = constraint_infos[i];
        auto lidar0 = lidars[info.lidar0_name];
        auto pcd0 = lidar0->getPCDInGlobal(info.pcd0_name);
        auto lidar1 = lidars[info.lidar1_name];
        auto pcd1 = lidar1->getPCDInGlobal(info.pcd1_name);
        
        pcl::IndicedMerge<PointT> merged;
        merged.push_back(info.lidar0_name, pcd0);
        merged.push_back(info.lidar1_name, pcd1);
        
        pcl::KdTreeFLANN<PointT> kd_merged;
        kd_merged.setInputCloud(merged.merged_cloud);
        
        std::unordered_map<int, int> idx_pairs;

        searchInterSection(pcd0, pcd1, find_intersection_threshold, idx_pairs);
        
        for (const auto& [id0_, id1_] : idx_pairs)
        {
            int id0 = merged.idxLocal2Global(info.lidar0_name, id0_);
            int id1 = merged.idxLocal2Global(info.lidar1_name, id1_);


            ConstCloudPtr merged_cloud = merged.merged_cloud;
            pcl::Indices indices;
            std::vector<float> dists;
            kd_merged.radiusSearch(merged_cloud->at(id0), plane_serach_radius, indices, dists);
            Eigen::Vector3f normal;
            double plane_score = checkPlane<PointT>(merged_cloud, indices, normal);
            if (threshold_plane < plane_score){
                if (lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() &&
                        lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end()){
                    normal = Eigen::Vector3f::UnitZ();
                    normal.normalize();
                }

                double distance = normal.dot(pcd0->at(id0_).getVector3fMap() - pcd1->at(id1_).getVector3fMap());
                if (distance < project_distance_threshold){

                    auto& detal = constraint_details[i].emplace_back();
                    detal.id0 = id0_;
                    detal.id1 = id1_;
                    detal.distance = std::abs(distance);
                    detal.normal = normal.cast<double>();

                    if ((lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() &&
                        lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end()) || 
                        pcd0->at(id0_).getVector3fMap().norm() < 5){
                        graph.emplace_shared<ml_calib::LidarJointOptFactor>(lidar0->symbol, lidar1->symbol, ground_noise_model, 
                                                                            pcd0->at(id0_).getVector3fMap().cast<double>(),
                                                                            pcd1->at(id1_).getVector3fMap().cast<double>(),
                                                                            normal.cast<double>(), true, info.weight);

                        detal.label = "ground";
                    }else{
                        graph.emplace_shared<ml_calib::LidarJointOptFactor>(lidar0->symbol, lidar1->symbol, plane_noise_model, 
                                                                            pcd0->at(id0_).getVector3fMap().cast<double>(),
                                                                            pcd1->at(id1_).getVector3fMap().cast<double>(),
                                                                            normal.cast<double>(), true);
                        detal.label = "plane";
                    }
                    
                }
            }
        }
    }
    for (const auto& fixed_lidar : fixed_lidars)
    {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(lidars[fixed_lidar]->symbol, gtsam::Pose3::Identity(), gtsam::noiseModel::Constrained::All(6)));
    }

    gtsam::Values initial;
    for (const auto& [lidar_name, lidar] : lidars)
    {
        if (lidar->initialized){
            initial.insert(lidar->symbol, gtsam::Pose3::Identity());
        }
    }
    gtsam::LevenbergMarquardtParams params;
    // params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    double init_error = optimizer.error();
    std::cout << "Initilization cost: " << init_error << std::endl;
    gtsam::Values result_ = optimizer.optimize();
    double opted_error = optimizer.error();
    std::cout << "Optimized cost: " << opted_error << std::endl;
    std::cout << "Cost Change: " << init_error - opted_error << std::endl;
    
    for (const auto& [lidar_name, lidar] : lidars)
    {
        if (lidar->initialized){
            lidar->pose = lidar->correct_mat.inverse() * result_.at<gtsam::Pose3>(lidar->symbol).matrix().cast<double>() * lidar->getPose();
        }
    }
}


std::vector<MLiDARConstraintDetail> MLCalib::getConstraintDetails(int id)
{
    if (id < 0 || id >= constraint_details.size()){
        throw std::runtime_error("Constraint ID out of range: " + std::to_string(id));
    }
    return constraint_details[id];
}

struct TmpVoxelInfo{
    int idx0, idx1;
    double dist;
};

void MLCalib::searchInterSection(ConstCloudPtr pcd0, 
                        ConstCloudPtr pcd1, 
                        double threshould,
                        std::unordered_map<int, int>& idx_pairs)
{
    idx_pairs.clear();

    std::unordered_map<VoxelIndex, std::vector<int>, VoxelIndexHash> voxel_map;
    std::unordered_map<VoxelIndex, TmpVoxelInfo, VoxelIndexHash> voxel_occupancied;

    for (int i = 0; i < pcd0->size(); ++i) {
        auto point = pcd0->points[i];
        // 计算点所属体素索引
        VoxelIndex idx;
        idx.x = static_cast<int>(std::floor(point.x / octree_resolution));
        idx.y = static_cast<int>(std::floor(point.y / octree_resolution));
        idx.z = static_cast<int>(std::floor(point.z / octree_resolution));

        // 如果体素不存在，则创建新点云
        if (voxel_map.find(idx) == voxel_map.end()) {
            voxel_map[idx] = std::vector<int>();
        }

        // 添加点到对应体素点云
        voxel_map[idx].push_back(i);
    }

    // OctreeT pcd0_octree(octree_resolution);
    // pcd0_octree.setInputCloud(pcd0);
    // pcd0_octree.addPointsFromInputCloud();

    pcl::KdTreeFLANN<PointT> pcd0_kd;
    pcd0_kd.setInputCloud(pcd0);

    pcl::KdTreeFLANN<PointT> pcd1_kd;
    pcd1_kd.setInputCloud(pcd1);

    for (const auto& [voxel_idx, point_indices] : voxel_map) {
        // PointT voxel_center;
        // voxel_center.x = (voxel_idx.x + 0.5) * octree_resolution;
        // voxel_center.y = (voxel_idx.y + 0.5) * octree_resolution;
        // voxel_center.z = (voxel_idx.z + 0.5) * octree_resolution;
        // // 在体素中心点上进行半径搜索
        // pcl::Indices point_indices;
        // std::vector<float> distances;
        // pcd0_kd.radiusSearch(voxel_center, 2 * octree_resolution, point_indices, distances);

    // for (auto leaf_it = pcd0_octree.leaf_begin(); leaf_it != pcd0_octree.leaf_end(); ++leaf_it) {
        // std::vector<int> point_indices;
        // leaf_it.getLeafContainer().getPointIndices(point_indices);

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
            for (int i = 0; i < 2; i++)
            {
                pcl::Indices indices_;
                std::vector<float> dists;
                pcd0_kd.radiusSearch(idx_pcd0, octree_resolution, indices_, dists);
                for (const auto& idx : indices_)
                {
                    PointT pt = pcd0->points[idx];
                    if (pcl::isFinite(pt)){
                        pcl::Indices indices;
                        std::vector<float> distance;
                        pcd1_kd.nearestKSearch(pt, 1, indices, distance);
                        if (distance[0] < min_dist){
                            min_dist = distance[0];
                            idx_pcd1 = indices[0];
                            idx_pcd0 = idx;
                        }
                    }
                }
            }
            auto point = pcd0->points[idx_pcd0];
            VoxelIndex idx;
            idx.x = static_cast<int>(std::floor(point.x / (1.2 * octree_resolution)));
            idx.y = static_cast<int>(std::floor(point.y / (1.2 * octree_resolution)));
            idx.z = static_cast<int>(std::floor(point.z / (1.2 * octree_resolution)));
            if (voxel_occupancied.find(idx) != voxel_occupancied.end()){
                if (voxel_occupancied[idx].dist > min_dist){
                    idx_pairs.erase(voxel_occupancied[idx].idx0);
                    idx_pairs[idx_pcd0] = idx_pcd1;

                    voxel_occupancied[idx].idx0 = idx_pcd0;
                    voxel_occupancied[idx].idx1 = idx_pcd1;
                    voxel_occupancied[idx].dist = min_dist;
                }
            }else{

                idx_pairs[idx_pcd0] = idx_pcd1;
                voxel_occupancied[idx].idx0 = idx_pcd0;
                voxel_occupancied[idx].idx1 = idx_pcd1;
                voxel_occupancied[idx].dist = min_dist;
            }
        }
    }
}

Eigen::Affine3d MLCalib::groundOptimization(CloudPtr pcd, std::set<int>& ground_idx, double height)
{
    gtsam::NonlinearFactorGraph graph;
    gtsam::Symbol key('p', 0);

    for (auto idx: ground_idx)
    {
        // gtsam::SharedNoiseModel noise_model;
        // // if (pcd->at(idx).z > 1.5){
        // //     noise_model = gtsam::noiseModel::mEstimator::Tukey::Create(4.685);
        // // }else{
        //     noise_model = gtsam::noiseModel::Robust::Create(
        //         gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        //         gtsam::noiseModel::Isotropic::Sigma(1, 0.1)
        //     );
        // // }
        // if (pcd->at(idx).getVector3fMap().norm() < 10){
            graph.emplace_shared<ml_calib::LiDARGroundPointFactor>(
                key, height, (Eigen::Vector3d() << pcd->at(idx).getVector3fMap().cast<double>()).finished(), plane_noise_model);
        // }
    }
    
    gtsam::Values initial;
    initial.insert(key, gtsam::Vector3(0,0,0));

    gtsam::LevenbergMarquardtParams params;

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values result_ = optimizer.optimize();

    Eigen::Vector3d result = result_.at<gtsam::Vector3>(key);
    
    Eigen::Affine3d trans_mat = Eigen::Affine3d::Identity();
    trans_mat.translate(Eigen::Vector3d(0, 0, result.z()));
    trans_mat.rotate(Eigen::AngleAxisd(result.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(result.x(), Eigen::Vector3d::UnitX()));
    return trans_mat;
}


void MLCalib::groundOpt(const std::string& lidar_name, const std::string& source_name)
{
    if (lidars.find(lidar_name) == lidars.end()){
        throw std::runtime_error("Lidar not found: " + lidar_name);
    }
    auto& lidar = lidars[lidar_name];
    if (lidar->source_pcds.find(source_name) == lidar->source_pcds.end()){
        throw std::runtime_error("Source point cloud not found: " + source_name);
    }
    CloudPtr pcd_tarnsformed = lidar->getPCDInGlobal(source_name);
    if (pcd_tarnsformed->empty()){
        throw std::runtime_error("Point cloud is empty: " + source_name);
    }

    // pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test3.pcd", *pcd_tarnsformed);
    Eigen::Affine3d trans = groundOptimization(pcd_tarnsformed, lidar->ground_indices[source_name], -root_lidar_height);
    // std::cout << trans.matrix() << std::endl;
    CloudPtr pcd_transed = pcl::make_shared<PointCloud>();
    pcl::transformPointCloud(*pcd_tarnsformed, *pcd_transed, trans.matrix().cast<float>());
    // pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test4.pcd", *pcd_tarnsformed);
    // std::cout << "euler: " << trans.rotation().eulerAngles(0, 1, 2 ).transpose();
    // std::cout << "trans: " << trans.translation().transpose();
    lidar->pose = trans.matrix() * lidar->pose.matrix();
}



void MLCalib::travelseLidarTree(const std::shared_ptr<MLiDAR>& lidar, const std::function<void(const std::shared_ptr<MLiDAR>&)>& func)
{
    func(lidar);
    for (const auto& leaf : lidar->target_leaves)
    {
        this->travelseLidarTree(leaf.target_lidar, func);
    }
}

void MLCalib::updateInitPose()
{
    auto root = lidars[root_lidar_name_];
    auto func = [this](const std::shared_ptr<MLiDAR>& lidar) {
        for (const auto& leaf: lidar->target_leaves)
        {
            if (leaf.target_lidar->initialized){
                continue; // 已经初始化过了
            }
            if (std::find(fixed_lidars.begin(), fixed_lidars.end(), leaf.target_lidar->name) != fixed_lidars.end()){
                continue; // 已经是固定的lidar
            }

            CloudPtr target_pcd = lidar->getPCDInGlobal(leaf.source_name);
            CloudPtr source_pcd = leaf.target_lidar->source_pcds[leaf.target_name];
            CloudPtr aligned_pcd = pcl::make_shared<PointCloud>();
            
            reg.setInputCloud(source_pcd);
            reg.setInputTarget(target_pcd);
            reg.align(*aligned_pcd);
            Eigen::Matrix4f trans = reg.getFinalTransformation();
            leaf.target_lidar->pose = trans.cast<double>();
            groundOpt(leaf.target_lidar->name, leaf.target_name);
            leaf.target_lidar->initialized = true;
        }
    };

    travelseLidarTree(root, func);
}

bool MLCalib::step(bool only_ground)
{
    std::cout << "MLCalib step: " ;
    if (!only_ground){
        std::cout << "joint plane optimization" << std::endl;
        buildConstraints();
    }else{
        std::cout << "joint ground optimization" << std::endl;
        buildGroundConstraints();
    }
    return true;
}

} // namespace ml_calib


