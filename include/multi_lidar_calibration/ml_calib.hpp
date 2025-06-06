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
#include <stdexcept>
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

    void push_back(const std::string& name, CloudPtr cloud);

    void addTargetLeaf(const std::string& source_name, const std::shared_ptr<MLiDAR>& target_lidar, const std::string& target_name);

    CloudPtr getPCDInGlobal(const std::string& pcd_name);

    void correctPose(const Eigen::Matrix4d& mat);

    void correctPose(const Eigen::Vector3d& euler, const Eigen::Vector3d& translation);

    CloudPtr operator[](const std::string& name);
 
    Eigen::Matrix4d getPose(){ return correct_mat * pose; }

    std::map<std::string, CloudPtr> source_pcds;

    std::map<std::string, std::set<int>> ground_indices;

    static patchwork::Params patchwork_parameters;

private:

    std::string name;

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    
    std::vector<std::shared_ptr<const MLiDAR>> constraint_lidars;

    std::vector<MLiDARLeaf> target_leaves;

    gtsam::Symbol symbol;

    Eigen::Matrix4d correct_mat = Eigen::Matrix4d::Identity();

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

    void addLiDAR(const std::string& name);
    
    bool step(bool only_ground);

    std::shared_ptr<MLiDAR>& operator[](const std::string& name);

    void setRootLidar(const std::string& lidar_name);

    void setRootLidarHeight(double height);

    void setLidarPose(const std::string& name, const Eigen::Matrix4d& pose);

    void setLidarFixed(const std::string& name, bool fixed);

    int addConstraint(const std::string& lidar0_name, const std::string& pcd0_name,
                    const std::string& lidar1_name, const std::string& pcd1_name, double weight = 1.0);

    void updateWeight(const int id, double weight);

    void buildGroundConstraints();

    void buildConstraints();

    std::vector<MLiDARConstraintDetail> getConstraintDetails(int id);

    void updateInitPose();

    void correctMapPrePose(const Eigen::Vector3d& map_correct_rot, const Eigen::Vector3d& map_correct_trans)
    {
        for (const auto [name, lidar] : lidars)
        {
            lidar->correctPose(map_correct_rot, map_correct_trans);
            lidar->correctPose(map_correct_rot, map_correct_trans);
            lidar->correctPose(map_correct_rot, map_correct_trans);
            lidar->correctPose(map_correct_rot, map_correct_trans);
        }
    }


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
                            std::unordered_map<int, int>& idx_pairs);

    Eigen::Affine3d groundOptimization(CloudPtr pcd, std::set<int>& ground_idx, double height);

    void groundOpt(const std::string& lidar_name, const std::string& source_name);

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

