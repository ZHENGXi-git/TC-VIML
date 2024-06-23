#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

#define PI 3.1415926
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class Line2D
{
public:
    Eigen::Vector2d PtrStart, PtrEnd, LineVec, Direction, PtrMid;
    Eigen::Vector3d hPtrStart, hPtrEnd;
    double Length;
    // Ax + By + C = 0;
    double A, B, C, A2B2;
    double Gradient;

    // the end points in the normalized camera frame
    Eigen::Vector4d LineObsN;

    Line2D(){};
    ~Line2D(){};
    Line2D(const Eigen::Vector4d &Vec, const Eigen::Matrix3d &K);
    Line2D(const Eigen::Vector4d &Vec);
    Eigen::Vector3d ComputeCircleNormal(const Eigen::Matrix3d &K);
    Eigen::Vector2d Point2Flined(const Eigen::Vector2d &minP);
};

class Line3D
{
public:
    Eigen::Vector3d PtrStart, PtrEnd, LineVec, Direction;
    double Length;

    Line3D(){};
    Line3D(const Vector6d &Vec);
    Line3D(const Eigen::Vector3d &ptrStart, const Eigen::Vector3d &ptrEnd);
    Line3D Transform3D(const Eigen::Matrix3d &R, const Eigen::Vector3d &t);
    Line2D Project3D(const Eigen::Matrix3d &K);

};

class PairsMatch
{
public:
    int index;
    Line3D line3dBodyP;
    Line3D line3dcamP;
    Line2D projectedlineP;
    Line2D detectedlineP;

    bool repetitive = false;
    Vector6d criteria;
    double weight;

    PairsMatch(int idx, Line3D &lineB, Line3D &lineC, Line2D &projectedL, Line2D &detectedL);

};

bool Compare(PairsMatch pair1, PairsMatch pair2);


class lineFeaturePerFrame
{
public:
    lineFeaturePerFrame(const Eigen::Vector4d &_line)
    {
        line = Line2D(_line); 
        use_flag = false;
        credible_line = true;
    }
    Line2D line;
    Line3D lineWorld; // the matching 3d line in this frame
    Line3D line3dCam;  // update after every optimization
    Line2D projectedLine; // update after every optimization
    Matrix3d A;
    Vector3d b;
    float errA;    //[0]
    float errD;    //[1]
    float overlap; //[2]
    bool use_flag;

    bool credible_line;

};

class lineFeaturePerId
{
public:
    const int feature_id;
    int start_frame;
    vector<lineFeaturePerFrame> linefeature_per_frame;

   vector<float> statistics_error_per_id;

    int used_num; 
    int all_obs_cnt;
    int solve_flag;

    bool credible_matching;

    lineFeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), solve_flag(0), credible_matching(true)
    {
    }

    int endFrame();

};


class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

class FeaturePerId
{
  public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();  // clear all features

    int getFeatureCount();

    int getLineFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);

    bool addFeaturesCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                                  const vector<pair<int, Eigen::Vector4d>> &lines, double td);

    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();

    void removeLineOutlier();

    float lineDiff(Line3D L1, Line3D L2);

    list<FeaturePerId> feature;
    int last_track_num;

    // line feature function
    int min_line_num = 3;
    list<lineFeaturePerId> linefeature;


  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif