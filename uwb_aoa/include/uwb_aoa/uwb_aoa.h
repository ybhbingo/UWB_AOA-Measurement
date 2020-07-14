#ifndef __UWB_AOA_H
#define __UWB_AOA_H

#include <cmath>
#include <stdlib.h>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


class uwb_aoa_t
{
public:
    uwb_aoa_t();
    ~uwb_aoa_t();
    void spin();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_tag;
    ros::Subscriber sub_station;
    ros::Publisher pub_global_pose;
    ros::Publisher pub_global_pose_noise;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    geometry_msgs::Point uwb_aoa_tag;
    geometry_msgs::Pose2D uwb_aoa_station;

    geometry_msgs::Point tag_in_robot;
    geometry_msgs::Point tag_in_global;

    unsigned char tag_lost_counter;
    unsigned char station_lost_counter;
    bool tag_lost_flag;
    bool station_lost_flag;
    bool tag_sub_start;
    bool station_sub_start;
    double theta_bias;

    double gaussian_noise();
    void sub_tag_callback(const geometry_msgs::Point& msg);
    void sub_station_callback(const geometry_msgs::Pose2D& msg);

};


#endif

