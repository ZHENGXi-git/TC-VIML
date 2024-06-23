
#include "line_pose_local_parameterization.h"

bool LinePoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    // x: state ==> x, y, z, w, x, y, z
    // delta
    x_plus_delta[0] = x[0] + delta[0];
    x_plus_delta[1] = x[1] + delta[1];
    x_plus_delta[2] = x[2] + delta[2];

    const double norm_delta = sqrt(delta[3] * delta[3] + delta[4] * delta[4] + delta[5] * delta[5]);
    if (norm_delta > 0.0)
    {
        const double sin_delta_by_delta = sin(norm_delta) / norm_delta;
        double q_delta[4];
        q_delta[0] = cos(norm_delta);
        q_delta[1] = sin_delta_by_delta * delta[3];
        q_delta[2] = sin_delta_by_delta * delta[4];
        q_delta[3] = sin_delta_by_delta * delta[5];
        //    QuaternionProduct(q_delta, x, x_plus_delta);
        x_plus_delta[3] = q_delta[0] * x[3] - q_delta[1] * x[4] - q_delta[2] * x[5] - q_delta[3] * x[6];
        x_plus_delta[4] = q_delta[0] * x[4] + q_delta[1] * x[3] + q_delta[2] * x[6] - q_delta[3] * x[5];
        x_plus_delta[5] = q_delta[0] * x[5] - q_delta[1] * x[6] + q_delta[2] * x[3] + q_delta[3] * x[4];
        x_plus_delta[6] = q_delta[0] * x[6] + q_delta[1] * x[5] - q_delta[2] * x[4] + q_delta[3] * x[3];
    }
    else
    {
        for (int i = 3; i < 7; ++i)
        {
            x_plus_delta[i] = x[i];
        }
    }

    return true;
}

bool LinePoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
