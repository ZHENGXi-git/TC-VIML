#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/matx.hpp>
#include <opencv2/core/matx.hpp>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/line_descriptor.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern cv::Mat IntrinsicMatrix;
extern cv::Mat Distortion;
extern Eigen::Matrix3d K;
extern int width;
extern int height;
extern int undistortion;
extern int LengthThreshold;

extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
