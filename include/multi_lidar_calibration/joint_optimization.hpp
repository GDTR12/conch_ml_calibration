#pragma once
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/GeneralSFMFactor.h>


namespace ml_calib
{

class LidarJointOptFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
private:
    Eigen::Vector3d p0_, p1_, n_;
    double factor_weight_;
    bool use_normal_;
public:
    LidarJointOptFactor(gtsam::Key key0, 
                        gtsam::Key key1, 
                        const gtsam::SharedNoiseModel& model,
                        const Eigen::Vector3d& p0, 
                        const Eigen::Vector3d& p1, 
                        const  Eigen::Vector3d& normal, 
                        bool use_normal = true,
                        const double factor_weight = 1)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key0, key1), p0_(p0), p1_(p1), n_(normal), use_normal_(use_normal), factor_weight_(factor_weight){}
    ~LidarJointOptFactor(){}

    gtsam::Vector evaluateError(const gtsam::Pose3& pose0, 
                                const gtsam::Pose3& pose1, 
                                boost::optional<gtsam::Matrix&> H0 = boost::none, 
                                boost::optional<gtsam::Matrix&> H1 = boost::none) const override 
    {
        gtsam::Vector ret(1);
        if (use_normal_){
            ret(0) = n_.transpose() * (pose0.rotation().matrix() * p0_ - pose1.rotation().matrix() * p1_ \
                    + pose0.translation() - pose1.translation());
            if (H0){
                Eigen::Matrix<double ,1, 6> jac0;
                jac0 << - n_.transpose() * pose0.rotation().matrix() * gtsam::skewSymmetric(p0_), 
                    n_.transpose();
                *H0 = jac0;
            }
            if (H1){
                Eigen::Matrix<double ,1, 6> jac0;
                jac0 << n_.transpose() * pose1.rotation().matrix() * gtsam::skewSymmetric(p1_), 
                    -n_.transpose();
                *H1 = jac0;
            }
        }else{
            Eigen::Vector3d dist = pose0.rotation().matrix() * p0_ - pose1.rotation().matrix() * p1_ \
                    + pose0.translation() - pose1.translation();
            ret(0) = dist.norm();

            Eigen::Matrix<double, 1, 3> pre_d = (1.0 / (ret(0))) * dist.transpose();

            if (H0){
                Eigen::Matrix<double, 1, 6> jac;
                jac << pre_d * ( - pose0.rotation().matrix() * gtsam::skewSymmetric(p0_)), (1.0 / ret(0)) * dist.transpose();
                *H0 = jac;
            }
            if (H1){
                Eigen::Matrix<double, 1, 6> jac;
                jac << pre_d * (pose1.rotation().matrix() * gtsam::skewSymmetric(p1_)), - (1.0 / ret(0)) * dist.transpose();
                *H1 = jac;
            }
        }
        ret *= factor_weight_;
        return ret;
    }
};


    
} // namespace ml_calib


