
#ifndef CATKIN_TCVINS_LINE_PROJECTION_FACTOR_H
#define CATKIN_TCVINS_LINE_PROJECTION_FACTOR_H

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "line_pose_local_parameterization.h"
#include "pose_local_parameterization.h"

class LineProjectionFactor: public ceres::SizedCostFunction<2, 7> // 2: residual; 7: state
{
public:
    LineProjectionFactor(const Eigen::Vector3d &_pts_start,
                         const Eigen::Vector3d &_pts_end,
                         const Eigen::Vector3d &_line_param,
                         const Eigen::Matrix3d _K,
                         const Eigen::Matrix3d _b_c_R, const Eigen::Vector3d _b_c_T);  // state, measurements param and intermediate parameters

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    // the calculated parameters
    Eigen::Vector3d pts_start;
    Eigen::Vector3d pts_end;
    Eigen::Vector3d line_param;
    Eigen::Matrix3d K;
    Eigen::Matrix3d b_c_R;
    Eigen::Vector3d b_c_T;

    static double sum_t;
};


#endif //CATKIN_TCVINS_LINE_PROJECTION_FACTOR_H
