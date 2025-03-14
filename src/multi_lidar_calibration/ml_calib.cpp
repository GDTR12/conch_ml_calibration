#include "multi_lidar_calibration/ml_calib.hpp"
#include "front_end/gem/lineplane_extractor.h"
#include "patchwork/patchworkpp.h"
#include "bag2pcd/bag2pcd.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace ml_calib
{

MLCalib::MLCalib()
{

}

MLCalib::~MLCalib(){}


void MLCalib::push_back(ConstCloudPtr pcd){pcds.push_back(pcd);}

void MLCalib::run()
{
    CloudPtr merged_pcd = pcl::make_shared<PointCloud>();
    for (ConstCloudPtr pcd: pcds)
    {
        *merged_pcd += *pcd;
    }

    std::vector<std::vector<int>> indices_list;
    int pre_idx = 0;
    for (size_t i = 0; i < pcds.size(); i++)
    {
        std::vector<int>& indices = indices_list.emplace_back();
        for (size_t j = 0; j < pcds[i]->size(); j++)
        {
            indices.push_back(j + pre_idx);
        }
        pre_idx += pcds[i]->size();
    }

    patchwork::Params patchwork_parameters;
    patchwork_parameters.verbose = true;
    patchwork::PatchWorkpp patchworkpp(patchwork_parameters);
   
    Eigen::MatrixXf cloud;
    cloud.resize(merged_pcd->size(), 4);
    for (size_t i = 0; i < merged_pcd->size(); i++)
    {
        cloud.row(i) << merged_pcd->at(i).x, merged_pcd->at(i).y, merged_pcd->at(i).z, merged_pcd->at(i).intensity;
    }
    patchworkpp.estimateGround(cloud);
    Eigen::VectorXi ground_idx    = patchworkpp.getGroundIndices();
    Eigen::VectorXi nonground_idx = patchworkpp.getNongroundIndices();
    std::vector<int> std_vec(ground_idx.data(), ground_idx.data() + ground_idx.size());

    pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/ground.pcd", *merged_pcd, std_vec);
    pcl::io::savePCDFile("/root/workspace/ros1/ml_bag/merged.pcd", *merged_pcd);


}

} // namespace ml_calib


