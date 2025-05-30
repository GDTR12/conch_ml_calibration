#pragma once
#include <memory>
#include <pcl/registration/registration.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "back_end/reglib.h"
#include <map>

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

class MLiDAR
{
public:
    MLiDAR(const std::string& lidar_name)
        : name(lidar_name){}

    MLiDAR() = delete;

    ~MLiDAR() = default;

    void push_back(const std::string& name, CloudPtr cloud)
    {
        source_pcds[name] = cloud;
    }

    void setFixed(bool fixed, Eigen::Matrix4d fix_transform = Eigen::Matrix4d::Identity())
    {
        this->fixed = fixed;
        this->fixed_transform = fix_transform;
    }

    void setTargetCloud(std::shared_ptr<const MLiDAR> target_lidar, const std::string& cloud_name)
    {
        if (canSource2FixedLiDAR()){
            
        }
        throw std::runtime_error("Cannot set target cloud for a non-fixed LiDAR.");
    }

    std::shared_ptr<MLiDAR> getTargetLiDAR()
    {
        return target_lidar;
    }

    bool canSource2FixedLiDAR()
    {
        if (fixed){
            return true;
        }
        std::shared_ptr<MLiDAR> current_lidar = getTargetLiDAR();
        while (true){
            if (current_lidar.get() == nullptr){
                return false;
            }
            if (current_lidar->fixed){
                return true;
            }
            current_lidar = current_lidar->getTargetLiDAR();
        }
    }

    void addConstraintLiDAR(const MLiDAR& lidar, const std::string& name)
    {

        // if ()
    }

    CloudPtr operator[](const std::string& name)
    {
        if (source_pcds.find(name) != source_pcds.end()){
            return source_pcds[name];
        }
        throw std::runtime_error("Source point cloud not found: " + name);
    }

private:

    std::string name;

    bool fixed = false;

    Eigen::Matrix4d fixed_transform = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

    std::map<std::string, CloudPtr> source_pcds;
    
    std::vector<std::shared_ptr<MLiDAR>> constraint_lidars;

    std::shared_ptr<MLiDAR> target_lidar;
};


class MLCalib
{
public:
    MLCalib();
    ~MLCalib();

    void addLiDAR(const std::string& name, std::shared_ptr<MLiDAR> lidar)
    {
        if (lidars.find(name) != lidars.end()){
            throw std::runtime_error("Lidar already exists: " + name);
        }
        lidars[name] = lidar;
    }
    
    void run();

    std::shared_ptr<MLiDAR>& operator[](const std::string& name)
    {
        if (lidars[name] != nullptr){
            return lidars[name];
        }
        throw std::runtime_error("Lidar not found: " + name);
    }

private:

    void preProcess();

    std::map<std::string, std::shared_ptr<MLiDAR>> lidars;

};

} // namespace ml_calib

