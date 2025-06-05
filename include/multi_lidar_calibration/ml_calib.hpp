#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/make_shared.h>
#include <pcl/pcl_base.h>
#include <pcl/registration/registration.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud.h>
#include "back_end/reglib.h"
#include <map>
#include <patchwork/patchworkpp.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "multi_lidar_calibration/ground_factor.hpp"
#include "multi_lidar_calibration/ground_joint_opt.hpp"
#include "multi_lidar_calibration/joint_optimization.hpp"

namespace pcl
{

// typedef PointXYZ PointSource;
// typedef PointXYZ PointTarget;
template <typename PointSource, typename PointTarget>
class G3Reg : public Registration<PointSource, PointTarget>
{
protected:
    using PointCloudSource = typename Registration<PointSource, PointTarget>::PointCloudSource;
    using PointCloudSourcePtr = typename PointCloudSource::Ptr;
    using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

    using PointCloudTarget = typename Registration<PointSource, PointTarget>::PointCloudTarget;
    using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
    using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

    using PointIndicesPtr = PointIndices::Ptr;
    using PointIndicesConstPtr = PointIndices::ConstPtr;

    using TargetGrid = VoxelGrid<PointTarget>;
    using TargetGridPtr = TargetGrid *;
    using TargetGridConstPtr = const TargetGrid *;
    // using TargetGridLeafConstPtr = typename TargetGrid::LeafConstPtr;

    void computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess) override
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr target = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::copyPointCloud(*this->input_, *source);
        pcl::copyPointCloud(*this->target_, *target);
        FRGresult solution = g3reg::GlobalRegistration(source, target);
        Eigen::Matrix4d tf = solution.tf;
        std::cout << solution.cluster_inliers << std::endl;
        pcl::transformPointCloud(*this->input_, output, tf);
        this->final_transformation_ = tf.cast<float>();
        this->converged_ = true;
    }

public:
    //   using Ptr = shared_ptr< G3Reg<PointSource, PointTarget> >;
    //   using ConstPtr = shared_ptr< const G3Reg<PointSource, PointTarget> >;

    G3Reg(){};
    ~G3Reg(){}

    void setG3RegParams(const std::string& url_path)
    {
        g3reg::config.load_config(url_path);
    }


protected:
    float resolution_;

};

// typedef pcl::PointXYZ PointT;
template<typename PointT>
class IndicedMerge
{
protected:

    using Cloud = pcl::PointCloud<PointT>;
    using CloudPtr = pcl::shared_ptr<Cloud>;
    using ConstCloudPtr = pcl::shared_ptr<const Cloud>;


    struct LocalPcdInfo{
        ConstCloudPtr cloud;
        int begin;
        int end;
        LocalPcdInfo(){begin = 0; end = 0;}
        LocalPcdInfo(int begin_idx, int end_idx, ConstCloudPtr pcd)
            : cloud(pcd), begin(begin_idx), end(end_idx){}
    };

    std::map<std::string, LocalPcdInfo> cloud_lst;

public:
    IndicedMerge(){}
    ~IndicedMerge(){}
    void push_back(const std::string name, ConstCloudPtr cloud)
    {
        if (cloud.get() == nullptr){
            return;
        }
        if (cloud->size() <= 0){
            return;
        }
        cloud_lst.emplace(name, LocalPcdInfo(merged_cloud->size(), merged_cloud->size() + cloud->size() - 1, cloud));
        *merged_cloud += *cloud;
    }

    CloudPtr merged_cloud = pcl::make_shared<Cloud>();

    ConstCloudPtr operator[](const std::string& name){return cloud_lst[name].cloud;}

    int idxLocal2Global(const std::string& name, int idx)
    {
        if (cloud_lst.find(name) != cloud_lst.end()){
            return idx + cloud_lst[name].begin;
        }else{
            return -1;
        }
    }

    std::pair<std::string, int> idxGlobal2Local(int idx)
    {
        for (const auto& pair: cloud_lst)
        {
            if (idx >= pair.second.begin && idx <= pair.second.end){
                return std::make_pair(pair.first, idx - pair.second.begin);
            }
        }
    }

    PointT& at(int idx)
    {
        return merged_cloud->at(idx);
    }
};


} // namespace name

namespace ml_calib
{

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using ConstCloud = const pcl::PointCloud<PointT>;
using CloudPtr = pcl::shared_ptr<PointCloud>;
using ConstCloudPtr = pcl::shared_ptr<const PointCloud>;
using OctreeT = pcl::octree::OctreePointCloud<PointT>;
using KdTree = pcl::KdTreeFLANN<PointT>;

class MLiDAR;
class MLCalib;

struct MLiDARLeaf
{
    std::string source_name;
    std::shared_ptr<MLiDAR> target_lidar;
    std::string target_name;
};

struct MLiDARConstraintInfo
{
    std::string lidar0_name;
    std::string pcd0_name;
    std::string lidar1_name;
    std::string pcd1_name;
    double weight = 1.0;
};

struct MLiDARConstraintDetail
{
    int id0;
    int id1;
    double distance;
    Eigen::Vector3d normal;
    std::string label;
};

class MLiDAR
{
public:
    MLiDAR(const std::string& lidar_name, int lidar_id = 0)
        : name(lidar_name) {
            symbol = gtsam::Symbol('l', lidar_id);
        }

    MLiDAR() = delete;

    ~MLiDAR() = default;

    void push_back(const std::string& name, CloudPtr cloud)
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

    void addTargetLeaf(const std::string& source_name, const std::shared_ptr<MLiDAR>& target_lidar, const std::string& target_name)
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

    CloudPtr getPCDInGlobal(const std::string& pcd_name)
    {
        if (source_pcds.find(pcd_name) != source_pcds.end()){
            CloudPtr pcd = source_pcds[pcd_name];
            CloudPtr transformed_pcd = pcl::make_shared<PointCloud>();
            pcl::transformPointCloud(*pcd, *transformed_pcd, pose.matrix().cast<float>());
            return transformed_pcd;
        }
        throw std::runtime_error("Source point cloud not found: " + pcd_name);
    }


    CloudPtr operator[](const std::string& name)
    {
        if (source_pcds.find(name) != source_pcds.end()){
            return source_pcds[name];
        }
        throw std::runtime_error("Source point cloud not found: " + name);
    }

    std::map<std::string, CloudPtr> source_pcds;

    std::map<std::string, std::set<int>> ground_indices;

    static patchwork::Params patchwork_parameters;

private:

    std::string name;

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    
    std::vector<std::shared_ptr<const MLiDAR>> constraint_lidars;

    std::vector<MLiDARLeaf> target_leaves;

    gtsam::Symbol symbol;

    bool initialized = false;

    friend class MLCalib;
};


// 定义体素索引结构体（用于哈希表键）
struct VoxelIndex {
    int x, y, z;

    // 必须重载==运算符用于哈希比较
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// 自定义哈希函数
struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& idx) const {
        // 使用质数乘法减少哈希冲突
        return ((std::hash<int>()(idx.x) ^ 
                (std::hash<int>()(idx.y) << 1)) >> 1) ^ 
                (std::hash<int>()(idx.z) << 1);
    }
};


class MLCalib
{
public:
    MLCalib(const std::string& g3reg_config_path);
    ~MLCalib();

    void addLiDAR(const std::string& name)
    {
        if (lidars.find(name) != lidars.end()){
            throw std::runtime_error("Lidar already exists: " + name);
        }
        std::shared_ptr<MLiDAR> lidar = std::make_shared<MLiDAR>(name, lidars.size());
        lidars[name] = lidar;
    }
    
    bool step(bool only_ground);

    std::shared_ptr<MLiDAR>& operator[](const std::string& name)
    {
        if (lidars[name] != nullptr){
            return lidars[name];
        }
        throw std::runtime_error("Lidar not found: " + name);
    }


    void setRootLidar(const std::string& lidar_name) //, double height = 1.2, const Eigen::Matrix4d& pose = Eigen::Matrix4d::Identity(), bool fixed = false)
    {
        if (lidars.find(lidar_name) == lidars.end()){
            throw std::runtime_error("Lidar not found: " + lidar_name);
        }
        root_lidar_name_ = lidar_name;
        groundOpt(lidar_name, lidars[lidar_name]->source_pcds.begin()->first);
    }
    
    void setRootLidarHeight(double height)
    {
        root_lidar_height = height;
    }

    void setLidarPose(const std::string& name, const Eigen::Matrix4d& pose)
    {
        if (lidars.find(name) == lidars.end()){
            throw std::runtime_error("Lidar not found: " + name);
        }
        lidars[name]->pose = pose;
        lidars[name]->initialized = true;
    }

    void setLidarFixed(const std::string& name, bool fixed)
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
    }


    int addConstraint(const std::string& lidar0_name, const std::string& pcd0_name,
                        const std::string& lidar1_name, const std::string& pcd1_name, double weight = 1.0)
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

    void updateWeight(const int id, double weight)
    {
        if (id < 0 || id >= constraint_infos.size()){
            throw std::runtime_error("Constraint ID out of range: " + std::to_string(id));
        }
        constraint_infos[id].weight = weight;
    }

    void buildGroundConstraints()
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
                if (pcd0->at(id0_).getVector3fMap().norm() > ground_radius_threshold){
                    continue;
                }
                int id0 = merged.idxLocal2Global(info.lidar0_name, id0_);
                int id1 = merged.idxLocal2Global(info.lidar1_name, id1_);


                ConstCloudPtr merged_cloud = merged.merged_cloud;
                pcl::Indices indices;
                std::vector<float> dists;
                kd_merged.radiusSearch(merged_cloud->at(id0), plane_serach_radius, indices, dists);
                Eigen::Vector3f normal;
                double plane_score = checkPlane<PointT>(merged_cloud, indices, normal);
                if (threshold_plane < plane_score){
                    if (lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() ||
                            lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end()){
                        normal = Eigen::Vector3f::UnitZ();
                        normal.normalize();
                    }

                    double distance = normal.dot(pcd0->at(id0_).getVector3fMap() - pcd1->at(id1_).getVector3fMap());
                    if (distance < project_distance_threshold){

 

                        if ((lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() ||
                            lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end())){
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
                std::cout << lidar->name << " " << lidar->symbol << std::endl;
                initial.insert(lidar->symbol, gtsam::Vector3(0,0,0));
            }
        }
        gtsam::LevenbergMarquardtParams params;
        params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;
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
                lidar->pose = t_f.matrix() * lidar->pose;
            }
        }
    }

    void buildConstraints()
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
                if (pcd0->at(id0_).getVector3fMap().norm() > ground_radius_threshold){
                    continue;
                }

                int id0 = merged.idxLocal2Global(info.lidar0_name, id0_);
                int id1 = merged.idxLocal2Global(info.lidar1_name, id1_);


                ConstCloudPtr merged_cloud = merged.merged_cloud;
                pcl::Indices indices;
                std::vector<float> dists;
                kd_merged.radiusSearch(merged_cloud->at(id0), plane_serach_radius, indices, dists);
                Eigen::Vector3f normal;
                double plane_score = checkPlane<PointT>(merged_cloud, indices, normal);
                if (threshold_plane < plane_score){
                    if (lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() ||
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

                        if ((lidar0->ground_indices[info.pcd0_name].find(id0_) != lidar0->ground_indices[info.pcd0_name].end() ||
                            lidar1->ground_indices[info.pcd1_name].find(id1_) != lidar1->ground_indices[info.pcd1_name].end())){
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
                std::cout << lidar->name << " " << lidar->symbol << std::endl;
                initial.insert(lidar->symbol, gtsam::Pose3::Identity());
            }
        }
        gtsam::LevenbergMarquardtParams params;
        params.verbosityLM = gtsam::LevenbergMarquardtParams::VerbosityLM::SUMMARY;
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
                lidar->pose = result_.at<gtsam::Pose3>(lidar->symbol).matrix().cast<double>() * lidar->pose;
            }
        }
    }


    std::vector<MLiDARConstraintDetail>& getConstraintDetails(int id)
    {
        if (id < 0 || id >= constraint_details.size()){
            throw std::runtime_error("Constraint ID out of range: " + std::to_string(id));
        }
        return constraint_details[id];
    }

    void updateInitPose();

private:

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

    void searchInterSection(ConstCloudPtr pcd0, 
                            ConstCloudPtr pcd1, 
                            double threshould,
                            std::unordered_map<int, int>& idx_pairs)
    {
        idx_pairs.clear();

        std::unordered_map<VoxelIndex, std::vector<int>, VoxelIndexHash> voxel_map;
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
                for (int i = 0; i < 3; i++)
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
                // std::cout << idx_pcd0 << " " << idx_pcd1 << std::endl;
                idx_pairs[idx_pcd0] = idx_pcd1;
            }
        }
    }

    Eigen::Affine3d groundOptimization(CloudPtr pcd, std::set<int>& ground_idx, double height)
    {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Symbol key('p', 0);

        for (auto idx: ground_idx)
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
                key, height, (Eigen::Vector3d() << pcd->at(idx).getVector3fMap().cast<double>()).finished(), noise_model);
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
    

    void groundOpt(const std::string& lidar_name, const std::string& source_name)
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

        pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test3.pcd", *pcd_tarnsformed);
        Eigen::Affine3d trans = groundOptimization(pcd_tarnsformed, lidar->ground_indices[source_name], -root_lidar_height);
        std::cout << trans.matrix() << std::endl;
        CloudPtr pcd_transed = pcl::make_shared<PointCloud>();
        pcl::transformPointCloud(*pcd_tarnsformed, *pcd_transed, trans.matrix().cast<float>());
        pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/test4.pcd", *pcd_tarnsformed);
        std::cout << "euler: " << trans.rotation().eulerAngles(0, 1, 2 ).transpose();
        std::cout << "trans: " << trans.translation().transpose();
        lidar->pose = trans.matrix() * lidar->pose.matrix();
    }

    void travelseLidarTree(const std::shared_ptr<MLiDAR>& lidar, const std::function<void(const std::shared_ptr<MLiDAR>&)>& func);

    pcl::G3Reg<PointT, PointT> reg;

    std::string g3reg_config_path_;

    std::map<std::string, std::shared_ptr<MLiDAR>> lidars;

    std::string root_lidar_name_;

    double root_lidar_height = 1.2; // 默认地面高度阈值

    std::vector<std::string> fixed_lidars;

    std::vector<MLiDARConstraintInfo> constraint_infos;

    std::vector<std::vector<MLiDARConstraintDetail>> constraint_details;

public:
    double find_intersection_threshold = 0.2; // 默认点云间交集搜索阈值

    double plane_serach_radius = 0.8; // 默认平面搜索半径

    double project_distance_threshold = 0.3; // 默认投影距离阈值

    double threshold_plane = 0.04;

    double ground_radius_threshold = 40;

    double octree_resolution = 1; // 默认八叉树分辨率
    
    gtsam::SharedNoiseModel ground_noise_model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Isotropic::Sigma(1, 0.1)
    );

    gtsam::SharedNoiseModel plane_noise_model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345),
        gtsam::noiseModel::Isotropic::Sigma(1, 0.5)
    );
};

} // namespace ml_calib

