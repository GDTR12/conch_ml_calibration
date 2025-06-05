#include "multi_lidar_calibration/ml_calib.hpp"
#include "bag2pcd/bag2pcd.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <memory>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ml_calib
{

MLCalib::MLCalib(const std::string &g3reg_config_path)
{
    g3reg_config_path_ = g3reg_config_path;
    reg.setG3RegParams(g3reg_config_path_);
}

MLCalib::~MLCalib(){}


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
    if (!only_ground){
        buildConstraints();
    }else{
        buildGroundConstraints();
    }
    return true;
}

} // namespace ml_calib


