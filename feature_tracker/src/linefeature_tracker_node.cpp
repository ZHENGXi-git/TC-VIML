//
// Created by zx.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "linefeature.h"

#define SHOW_UNDISTORTION 0

ros::Publisher pub_linefeature;

LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
double frame_cnt = 0;
bool init_pub = 0;
double sum_time = 0.0;
double mean_time = 0.0;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;

        return;
    }

    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    TicToc t_r;

    if (PUB_THIS_FRAME)
    {
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat show_img = ptr->image;
        frame_cnt++;
        trackerData.readImage(ptr->image.rowRange(0, ROW), img_msg->header.stamp.toSec());

        pub_count++;
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;
        sensor_msgs::ChannelFloat32 start_x;
        sensor_msgs::ChannelFloat32 start_y;
        sensor_msgs::ChannelFloat32 end_x;
        sensor_msgs::ChannelFloat32 end_y;

        feature_lines->header = img_msg->header;
        feature_lines->header.frame_id = "world";
        Vec4f line;
        geometry_msgs::Point32 p;

        auto &cur_lines = trackerData.cur_lines;
        auto &ids = trackerData.ids;

        for (unsigned int i = 0; i < cur_lines.size(); ++i)
        {
            {
                int l_id = ids[i];
                p.x = 0;
                p.y = 0;
                p.z = 1;
                feature_lines->points.push_back(p);
                line = cur_lines[i];
                id_of_line.values.push_back(l_id);
                start_x.values.push_back(line[0]);
                start_y.values.push_back(line[1]);
                end_x.values.push_back(line[2]);
                end_y.values.push_back(line[3]);

            }
        }
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(start_x);
        feature_lines->channels.push_back(start_y);
        feature_lines->channels.push_back(end_x);
        feature_lines->channels.push_back(end_y);


        if (!init_pub)
        {
            init_pub = 1;
        }
        else
        {
            pub_linefeature.publish(feature_lines);
        }

    }
    sum_time += t_r.toc();
    mean_time = sum_time / frame_cnt;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "linefeature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    ROS_INFO("start line feature");
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_linefeature = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);

    ros::spin();
    return 0;

}






















