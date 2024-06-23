#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <vector>
#include <cstdio>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include "factor/line_pose_local_parameterization.h"
#include "factor/line_projection_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

#define Sqrt2(x) x * x

struct Data
{
    Data(FILE *f)
    {
        if (fscanf(f, " %lf,%f,%f,%f,%f,%f,%f",
                   &t,
                   &px, &py, &pz,
                   &roll, &pitch, &yaw
        ) != EOF)
        {
            t /= 1;
        }
    }
    double t;
    float px, py, pz;
    float roll, pitch, yaw;
};

class Estimator
{
  public:
    Estimator();

    void setParameter();
    void setParameters(const string &calib_file, vector<Vector6d> &_lines3d);
    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    void processImagewithLine(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const cv::Mat &image_show,
    const vector<pair<int, Eigen::Vector4d>> &lines, const std_msgs::Header &header);

    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();  // add lines
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void solveOdometry();  // add lines
    void slideWindowNew();
    void slideWindowOld();
    
    void vector2double();
    void double2vector();
    bool failureDetection();

    void readBenchMark();
    int getGTIndex(float time_stamp, int preIndex);
    int BeginIndexSW = 0;

    std::vector<Data> benchmark;

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];   // the extrinsic from IMU(body) to camera
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td = 0;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;


    // for the line feature tightly coupled struction

    vector<Vector6d> lines3d_map;

    std_msgs::Header LineHeaders[(WINDOW_SIZE + 1)];
    cv::Mat images_show[WINDOW_SIZE + 1];

    vector<Line3D> lines3dBody[WINDOW_SIZE + 1];
    vector<Line3D> lines3dCam[WINDOW_SIZE + 1];   // 3D lines on the camera frame
    vector<Line2D> projectedLines[WINDOW_SIZE + 1];
    vector<Line2D> detectedLines[WINDOW_SIZE + 1];

    vector<PairsMatch> lineMatches[WINDOW_SIZE + 1];
    vector<PairsMatch> lineMatchesFilter[WINDOW_SIZE + 1];

    vector<Line3D> line3dBody;
    vector<Line3D> line3dCam;
    vector<Line2D> projectedline;
    vector<Line2D> detectedline;

    vector<PairsMatch> lineMatch;
    vector<PairsMatch> lineMatchFilter;

    vector<Line3D> LinesInFoV;
    vector<Line3D> WorldLinesInFOV[WINDOW_SIZE + 1];

    // process the line initialization after VINS finish initialization
    void initialLineFoVWindow();
    void updateLinePairInWindow();

    // update the line FoV of the 'idx' frame in this slide window
    void UpdateLinesInFoV(int idx, const Matrix3d &_Ric, const Vector3d &_Tic);

    void slideWindowWithLinesFoV();

    double CalAngleDist(const Line2D &_projectedL, const Line2D &_detectedL);
    Eigen::Vector2d CalEulerDist(const Line2D &_projectedL, const Line2D &_detectedL);

    // calculate the line pair of the detected lines in vector<lineFeaturePerFrame>
    pair<Eigen::Vector3f, pair<Line2D, Line3D>> LineCorrespondenceInFrame(const int &_frame_index,
                                                   const Line2D &_detectLine);

    Eigen::Vector4d CalAngleDistInCamera(const Line3D &_lineC, const Line2D &_projectedL,
                                         const Line2D &_detectedL);
    void LineCorrespondence();

    void CorrespondenceFilter();
    void updateLineConstraint();

    void OptimizationWithLine();

    void VisualizationMatchedLines();

    void VisualizationProLinesInFoV();

    // visual the frame_count image
    void VisualizationWithTrackLines();

    vector<double> ChiTestThreshold;

    cv::Mat cv_KMatrix, cv_dist;
    Eigen::Matrix3d K;
    int width, height;

    Eigen::Vector3d Tbw;
    Eigen::Matrix3d Rbw;   // from world to body0

    int undisKeyLine;

    // the threshold in line correspondence
    double overlap_th;
    double dist_th;
    double degree_th;
    double angle_th;

    double outlier_th;

    double lambda;
    double threshold;

    int per_inliers;  // min_matches

    int iterations;

    int save;


};
