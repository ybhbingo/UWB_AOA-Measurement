#include "uwb_aoa/uwb_aoa.h"

uwb_aoa_t::uwb_aoa_t()
{
    sub_tag = nh.subscribe("framework_in_global", 1, &uwb_aoa_t::sub_tag_callback, this);
    sub_station = nh.subscribe("robot_in_global", 1, &uwb_aoa_t::sub_station_callback, this);
    pub_global_pose_noise = nh.advertise<geometry_msgs::Point>("measured_target_position", 1);

    theta_bias = 7.0/180.0*M_PI;

    tag_lost_flag = false;
    station_lost_flag = false;
    tag_sub_start = false;
    station_sub_start = false;
}

uwb_aoa_t::~uwb_aoa_t()
{

}

void uwb_aoa_t::sub_tag_callback(const geometry_msgs::Point& msg)
{
    uwb_aoa_tag = msg;
    tag_lost_counter = 0;
    tag_sub_start = true;
}

void uwb_aoa_t::sub_station_callback(const geometry_msgs::Pose2D& msg)
{
    uwb_aoa_station = msg;

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(uwb_aoa_station.x, uwb_aoa_station.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, uwb_aoa_station.theta);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "global_frame", "robot_frame"));

    station_lost_counter = 0;
    station_sub_start = true;
}

void uwb_aoa_t::spin()
{
    static double distance = 0;
    static double theta = 0;
    static double theta_true = 0;
    geometry_msgs::PointStamped tag_in_robot_stamped;
    geometry_msgs::PointStamped tag_in_global_stamped;
    ros::Rate r(50);

    while(ros::ok())
    {
        ros::spinOnce();

        tag_lost_counter++;
        tag_lost_counter = tag_lost_counter >= 100 ? 100 : tag_lost_counter;
        tag_lost_flag = tag_lost_counter >= 100 ? true : false;

        station_lost_counter++;
        station_lost_counter = station_lost_counter >= 100 ? 100 : station_lost_counter;
        station_lost_flag = station_lost_counter >= 100 ? true : false;

        if( !tag_lost_flag && !station_lost_flag && tag_sub_start && station_sub_start )
        {
            distance = hypot(uwb_aoa_tag.x-uwb_aoa_station.x, uwb_aoa_tag.y-uwb_aoa_station.y);
            theta_true = atan2(uwb_aoa_tag.y-uwb_aoa_station.y, uwb_aoa_tag.x-uwb_aoa_station.x) - uwb_aoa_station.theta;
            theta = theta_true + gaussian_noise() + theta_bias;
            tag_in_robot.x = distance * cos(theta);
            tag_in_robot.y = distance * sin(theta);

            tag_in_robot_stamped.header.frame_id = "robot_frame";
            tag_in_robot_stamped.header.stamp = ros::Time();
            tag_in_robot_stamped.point = tag_in_robot;

            listener.transformPoint("global_frame", tag_in_robot_stamped, tag_in_global_stamped);

            tag_in_global = tag_in_global_stamped.point;
            pub_global_pose_noise.publish(tag_in_global);
        }
        r.sleep();
    }
}

double uwb_aoa_t::gaussian_noise()
{
    static double V1, V2, S;
    static int phase = 0;
    double theta_error_variance = 1.0/180.0*M_PI;
    double X;

    if ( phase == 0 )
    {
        do
        {
            double U1 = (double)rand()/RAND_MAX;
            double U2 = (double)rand()/RAND_MAX;

            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while ( S >= 1 || S == 0 );
        
        X = V1 * sqrt(-2 * log(S)/S);
    }
    else
    {
        X = V2 * sqrt(-2 * log(S)/S);
    }
    
    phase = 1 - phase;

    X = X * theta_error_variance;
    return X;
}


