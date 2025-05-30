#pragma once
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace ml_calib
{

class GroundJointOptFactor : public gtsam::NoiseModelFactor3<gtsam::Vector2, gtsam::Vector2, gtsam::Vector1, gtsam::Vector1>
{
private:
    gtsam::Vector3 p_, p1_;
    double factor_weight_;
public:

    GroundJointOptFactor(const gtsam::Vector3& p, 
                         const gtsam::Vector3& p1, 
                         gtsam::Key key0, 
                         gtsam::Key key1, 
                         gtsam::Key key2, 
                         gtsam::Key key3, 
                         const gtsam::SharedNoiseModel& model,
                         double factor_weight = 1)
        : gtsam::NoiseModelFactor3<gtsam::Vector2, gtsam::Vector2, gtsam::Vector1, gtsam::Vector1>(model, key0, key1, key2, key3), 
          p_(p), p1_(p1), factor_weight_(factor_weight) {}

    gtsam::Vector evaluateError(const gtsam::Vector2& params, 
                                const gtsam::Vector2& params1,
                                const gtsam::Vector1& params2,
                                const gtsam::Vector1& params3,
                                boost::optional<gtsam::Matrix&> H0 = boost::none, 
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none,
                                boost::optional<gtsam::Matrix&> H3 = boost::none) const override 
    {
        double x(p_.x()), y(p_.y()), z(p_.z());
        double alpha(params(0)), beta(params(1)), d(params2(0));
        double x1(p1_.x()), y1(p1_.y()), z1(p1_.z());
        double alpha1(params1(0)), beta1(params1(1)), d1(params3(0));

        gtsam::Vector error(1);
        error(0) = -x * sin(beta) + y * cos(beta) * sin(alpha) + z * cos(beta) * cos(alpha) + d 
                  + x1 * sin(beta1) - y1 * cos(beta1) * sin(alpha1) - z1 * cos(beta1) * cos(alpha1) - d1;
        if (H0){
            gtsam::Matrix jac(1, 2);
            jac(0) = y * cos(beta) * cos(alpha) - z * cos(beta) * sin(alpha);
            jac(1) = -x * cos(beta) - y * sin(beta) * sin(alpha) - z * sin(beta) * cos(alpha);
            *H0 = jac;
        }
        if (H1){
            gtsam::Matrix jac(1, 2);
            jac(0) = -(y1 * cos(beta1) * cos(alpha1) - z1 * cos(beta1) * sin(alpha1));
            jac(1) = -(-x1 * cos(beta1) - y1 * sin(beta1) * sin(alpha1) - z1 * sin(beta1) * cos(alpha1));
            *H1 = jac;
        }
        if (H2){
            gtsam::Matrix jac(1, 1);
            jac(0, 0) = 1;
            *H2 = jac;
        }

        if (H3){
            gtsam::Matrix jac(1, 1);
            jac(0, 0) = -1;
            *H3 = jac;
        }
        error *= factor_weight_;
        return error;
    }

};

}
