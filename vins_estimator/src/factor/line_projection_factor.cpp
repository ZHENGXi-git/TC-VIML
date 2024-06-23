
#include "line_projection_factor.h"

double LineProjectionFactor::sum_t;

LineProjectionFactor::LineProjectionFactor(const Eigen::Vector3d &_pts_start, const Eigen::Vector3d &_pts_end,
                                           const Eigen::Vector3d &_line_param, const Eigen::Matrix3d _K,
                                           const Eigen::Matrix3d _b_c_R, const Eigen::Vector3d _b_c_T)
                                           : pts_start(_pts_start), pts_end(_pts_end)
{
    line_param.x() = _line_param.x();   // A
    line_param.y() = _line_param.y();   // B
    line_param.z() = _line_param.z();   // C
    K = _K;
    b_c_R = _b_c_R;
    b_c_T = _b_c_T;
};

bool LineProjectionFactor::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    // Eigen::Vector3d Ti(parameters[0][0], parameters[0][1], parameters[0][2]);
    // Eigen::Quaterniond Qi(parameters[0][3], parameters[0][4], parameters[0][5], parameters[0][6]);

    double mea_u_start = 0.0;
    double mea_v_start = 0.0;
    double mea_u_end = 0.0;
    double mea_v_end = 0.0;

    Eigen::Matrix<double, 3, 1> T_w = Eigen::Matrix<double, 3, 1>(parameters[0][0], parameters[0][1], parameters[0][2]);
    // w x y z
    Eigen::Matrix<double, 3, 3> R_w =
            Eigen::Quaternion<double>(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]).normalized().toRotationMatrix();

    //  Eigen::Quaternion<double>(parameters[0][3], parameters[0][4], parameters[0][5], parameters[0][6]).normalized().toRotationMatrix();

    Eigen::Matrix<double, 3, 3> R_W_i = R_w;
    Eigen::Matrix<double, 3, 1> T_w_i = T_w;
    Eigen::Matrix<double, 3, 3> R = b_c_R.transpose() * R_W_i.transpose();   // trans to camera frame
    Eigen::Matrix<double, 3, 1> t = -R * T_w_i - (b_c_R.transpose() * b_c_T); //can be optimized

    Eigen::Vector3d pts_camera_start = R * pts_start + t;
    Eigen::Vector3d pts_camera_end = R * pts_end + t;

    const Eigen::Matrix<double, 3, 1> pts_start_image = K * pts_camera_start; //    K * (R * pts_start + t);
    const Eigen::Matrix<double, 3, 1> pts_end_image = K * pts_camera_end;  // (R * pts_end + t);

    double u_start = pts_start_image.x() / pts_start_image.z();
    double v_start = pts_start_image.y() / pts_start_image.z();
    double u_end = pts_end_image.x() / pts_end_image.z();
    double v_end = pts_end_image.y() / pts_end_image.z();

    double a = line_param.x();
    double b = line_param.y();
    double c = line_param.z();
    double d = a * a + b * b;

    mea_u_start = (b * b * u_start - a * b * v_start - a * c) / (d);
    mea_v_start = (a * a * v_start - a * b * u_start - b * c) / (d);
    mea_u_end = (b * b * u_end - a * b * v_end - a * c) / (d);
    mea_v_end = (a * a * v_end - a * b * u_end - b * c) / (d);

    Eigen::Map<Eigen::Vector2d> residual(residuals);


    double rho_line = 1.0;

    residual[0] = rho_line * sqrt((mea_u_start - u_start) * (mea_u_start - u_start) + (mea_v_start - v_start) * (mea_v_start - v_start));
    residual[1] = rho_line * sqrt((mea_u_end - u_end) * (mea_u_end - u_end) + (mea_v_end - v_end) * (mea_v_end - v_end));

    double lambda = 1;

    if (jacobians)
    {
        Eigen::Matrix<double, 1, 2> _e_p;
        double ep11 = -2 / d * ((mea_u_start - u_start) * a * a + a * b * (mea_v_start - v_start)) * lambda;
        double ep12 = -2 / d * ((mea_u_start - u_start) * a * b + b * b * (mea_v_start - v_start)) * lambda;

        double ep11_ = -2 / d * ((mea_u_end - u_end) * a * a + a * b * (mea_v_end - v_end)) * lambda;
        double ep12_ = -2 / d * ((mea_u_end - u_end) * a * b + b * b * (mea_v_end - v_end)) * lambda;

        _e_p << ep11, ep12;

        Eigen::Matrix<double, 1, 2> _e_p_;
        _e_p_ << ep11_, ep12_;

        double fx = K(0, 0);
        double fy = K(1, 1);
        double p_start_x_ = pts_camera_start(0);
        double p_start_y_ = pts_camera_start(1);
        double p_start_z_ = pts_camera_start(2);
        Eigen::Matrix<double, 2, 3> _p_p_s(2, 3);
        _p_p_s << fx / p_start_z_, 0, -fx * p_start_x_ / (p_start_z_ * p_start_z_),
                0, fy / p_start_z_, -fy * p_start_y_ / (p_start_z_ * p_start_z_);
        double p_end_x_ = pts_camera_end(0);
        double p_end_y_ = pts_camera_end(1);
        double p_end_z_ = pts_camera_end(2);
        Eigen::Matrix<double, 2, 3> _p_p_e(2, 3);
        _p_p_e << fx / p_end_z_, 0, -fx * p_end_x_ / (p_end_z_ * p_end_z_),
                0, fy / p_end_z_, -fy * p_end_y_ / (p_end_z_ * p_end_z_);

        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);

        Eigen::Matrix<double, 3, 6> jaco_s;
        jaco_s.leftCols<3>().setIdentity();
        jaco_s.rightCols<3>() = Utility::skewSymmetric(pts_camera_start);

        Eigen::Matrix<double, 3, 6> jaco_e;
        jaco_e.leftCols<3>().setIdentity();
        jaco_e.rightCols<3>() = Utility::skewSymmetric(pts_camera_end);

        jacobian_pose.block<1, 6>(0, 0) = _e_p * _p_p_s * jaco_s;
        jacobian_pose.block<1, 6>(1, 0) = _e_p_ * _p_p_e * jaco_e;
        jacobian_pose.rightCols<1>().setZero();

    }
    sum_t += tic_toc.toc();

    return true;
}

void LineProjectionFactor::check(double **parameters)
{

}