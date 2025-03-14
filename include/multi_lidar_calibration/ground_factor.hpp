#pragma once
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


namespace ml_calib
{

class LiDARGroundPointFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
private:
    double z_;
    Eigen::Vector3d p_;
public:
    gtsam::Vector evaluateError(const gtsam::Vector3& params, boost::optional<gtsam::Matrix&> H = boost::none) const override 
    {
        double x(p_.x()), y(p_.y()), z(p_.z());
        double alpha(params(0)), beta(params(1)), d(params(2));
        if (H){
            gtsam::Matrix jac(1, 3);
            jac(0) = y * cos(beta) * cos(alpha) - z * cos(beta) * sin(alpha);
            jac(1) = -x * cos(beta) - y * sin(beta) * sin(alpha) - z * sin(beta) * cos(alpha);
            jac(2) = 1;
            *H = jac;
        }
        gtsam::Vector error(1);
        error(0) = -x * sin(beta) + y * cos(beta) * sin(alpha) + z * cos(beta) * cos(alpha) + d - z_;
        return error;
    }
    LiDARGroundPointFactor(gtsam::Key key, double z, const Eigen::Vector3d& p, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, key), z_(z), p_(p){}
    ~LiDARGroundPointFactor(){}
};


    
} // namespace ml_calib


