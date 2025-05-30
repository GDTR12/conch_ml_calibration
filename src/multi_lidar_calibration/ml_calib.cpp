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
{}

MLCalib::~MLCalib(){}


void MLCalib::preProcess()
{
        addLiDAR("front", std::make_shared<ml_calib::MLiDAR>("front"));
        addLiDAR("left", std::make_shared<ml_calib::MLiDAR>("left"));
        addLiDAR("right", std::make_shared<ml_calib::MLiDAR>("right"));
        addLiDAR("back", std::make_shared<ml_calib::MLiDAR>("back"));
}

void MLCalib::run()
{
    preProcess();

}

} // namespace ml_calib


