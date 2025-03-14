#pragma once

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

class MLCalib
{
public:
    MLCalib();
    ~MLCalib();

    void push_back(ConstCloudPtr pcd);
    void run();

private:
    std::vector<ConstCloudPtr> pcds;
};

} // namespace ml_calib

