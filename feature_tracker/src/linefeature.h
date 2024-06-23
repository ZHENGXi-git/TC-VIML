
#ifndef SRC_LINEFEATURE_H
#define SRC_LINEFEATURE_H

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace camodocal;
using namespace ros;

Vec4f inBorder(const Vec4f &line);
void reduceVector2(vector<Vec4f> &v, vector<uchar> status);
void reduceVector2(vector<int> &v, vector<uchar> status);


class LineFeatureTracker
{
public:
    LineFeatureTracker();

    void readImage(const cv::Mat &_img, double _cur_time);
    void setMast();
    void addLines(vector<Vec4f> &lines);
    void updateIDs();
    void updateForwPts();
    void findForwMatchLine();


    cv::Mat mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<Vec4f>  cur_lines, forw_lines;
    Vec4f track_line;
    vector<cv::Point2f> cur_pts, forw_pts; // the end points of the lines
    vector<int> ids; // lines id
    vector<int> track_cnt;
    vector<Vec4f> prev_un_lines, cur_un_lines;
    map<int, Vec4f> cur_un_lines_map;
    map<int, Vec4f> prev_un_lines_map;
    camodocal::CameraPtr m_camera;

    vector<Vec4f> forw_detect_lines;
    vector<Vec4f> remain_detect_lines;

    vector<uchar> status;

    double cur_time;
    double prev_time;

    static int n_id_line;

    double sum_time;

};



#endif //SRC_LINEFEATURE_H
