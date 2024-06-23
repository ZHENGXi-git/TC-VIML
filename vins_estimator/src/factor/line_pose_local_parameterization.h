
#ifndef CATKIN_TCVINS_LINE_POSE_LOCAL_PARAMETERIZATION_H
#define CATKIN_TCVINS_LINE_POSE_LOCAL_PARAMETERIZATION_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class LinePoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

#endif //CATKIN_TCVINS_LINE_POSE_LOCAL_PARAMETERIZATION_H
