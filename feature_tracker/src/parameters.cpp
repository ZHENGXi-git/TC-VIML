#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;  // height
int COL;  // width
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

// for line feature detection
int width;
int height;
int undistortion;
int LengthThreshold;
cv::Mat IntrinsicMatrix;
cv::Mat Distortion;
Eigen::Matrix3d K;



template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    double m_fx = static_cast<double>(fsSettings["fx"]);
    double m_fy = static_cast<double>(fsSettings["fy"]);
    double m_cx = static_cast<double>(fsSettings["cx"]);
    double m_cy = static_cast<double>(fsSettings["cy"]);

    IntrinsicMatrix = (cv::Mat_<double>(3, 3) << m_fx, 0, m_cx, 0, m_fy, m_cy, 0, 0, 1);
    cv::cv2eigen(IntrinsicMatrix, K);

    double m_k1 = static_cast<double>(fsSettings["k1"]);
    double m_k2 = static_cast<double>(fsSettings["k2"]);
    double m_p1 = static_cast<double>(fsSettings["p1"]);
    double m_p2 = static_cast<double>(fsSettings["p2"]);

    Distortion = (cv::Mat_<double>(1, 4) << m_k1, m_k2, m_p1, m_p2);

    width = static_cast<int>(fsSettings["width"]);
    height = static_cast<int>(fsSettings["height"]);
    LengthThreshold = static_cast<int>(fsSettings["length_threshold"]);
    undistortion = static_cast<int>(fsSettings["undisKeyLine"]);

    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["height"];
    COL = fsSettings["width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();

    std::cout << "feature tracker read parameters----------------" << std::endl;
}
