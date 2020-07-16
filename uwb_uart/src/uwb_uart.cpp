#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include <stdlib.h>
#include <iostream>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

serial::Serial ros_ser;

// serialPort::UWBData sensor_data;
geometry_msgs::Pose2D tag_uwb;

double distance;

#define FRAME_1_LENGTH    13
#define FRAME_2_LENGTH    19 
#define FRAME_LENGTH_SUM  32
#define PI (3.1415)
void print_frame_data(uint8_t* data_addr, int frame_length)
{
	if( frame_length == FRAME_1_LENGTH )
	{
		unsigned int angle = data_addr[8] + (data_addr[9] << 8);
		unsigned int distance_cm = data_addr[10] + (data_addr[11] << 8);

        // sensor_data.distance = distance_cm;
        // sensor_data.angle = angle;

        distance = (double)distance_cm * 0.01;
        // tag_uwb.theta = -((double)angle - 90);
        // tag_uwb.theta = tag_uwb.theta >= 0 ? tag_uwb.theta : (tag_uwb.theta + 360);
        // tag_uwb.x = distance * cos(tag_uwb.theta / 180.0 * PI);
        // tag_uwb.y = distance * sin(tag_uwb.theta / 180.0 * PI);
        tag_uwb.theta = angle;
        tag_uwb.x = distance;
		// printf("Data Frame: \n");
		// printf("Frame Length :%d \n", frame_length);
		// printf("Relative Angle is: %d \n", angle);
        // printf("Distance is: %d \n", distance_cm);
        // ROS_INFO_STREAM(cos(tag_uwb.theta));
        // ROS_INFO_STREAM(sin(tag_uwb.theta));
	}
	else if ( frame_length == FRAME_2_LENGTH )
	{
		// printf("Status Frame: \n");
		// printf("First Path Receive Power: %d\n", data_addr[12]);
		// printf("Average Receive Power: %d\n", data_addr[13]);
        // printf("Battery Charge Left: %d\n", data_addr[16]);
		//printf();
	}
}

void get_frame_data(uint8_t* data_array, int data_length) 
{
	int i = data_length;
	int data_frame_flag = 0;
	int status_frame_flag = 0;
	for(i = (data_length - FRAME_LENGTH_SUM) ; i >= 0; --i)
	{
		if( data_array[i] == 0xAA && data_array[i+1] == 0x55 )
		{
			if( data_array[i+2] == 0x09 && data_array[i+3] == 0x3C )
			{
				if( data_array[i+FRAME_1_LENGTH] == 0xAA )
				{
					data_frame_flag = 1;
				}
			}
			else if( data_array[i+2] == 0x0F && data_array[i+3] == 0x46 )
			{
				if( data_array[i+FRAME_2_LENGTH] == 0xAA )
				{
					status_frame_flag = 1;
				}
			}
		}
		if( data_frame_flag )
		{
			print_frame_data(&(data_array[i]), FRAME_1_LENGTH);
			data_frame_flag = 0;
			break;
		}
		else if ( status_frame_flag )
		{
			print_frame_data(&(data_array[i]), FRAME_2_LENGTH);
			status_frame_flag = 0;
			break;
		}
	}
}

uint8_t data_rcv[256] = {0};

int main (int argc, char** argv)
{
    int i;
    static int p;
 
    std_msgs::UInt8MultiArray  r_buffer;
    ros::init(argc, argv, "uwb_uart_node");
    ros::NodeHandle nh_;
    
    ros::Publisher sensor_pub = nh_.advertise<geometry_msgs::Pose2D>("/uwb_input", 1, true);
    
    try
    {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if(ros_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(10);

    int p_max = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        if(ros_ser.available())
        {
            // ROS_INFO_STREAM("Reading from serial port");
            //std_msgs::String serial_data;
            std_msgs::UInt8MultiArray  serial_data;
            p = ros_ser.available();
            ros_ser.read (serial_data.data, p);
            if( p > FRAME_LENGTH_SUM )
            {
                get_frame_data((uint8_t*)&serial_data.data[0], p);
                sensor_pub.publish(tag_uwb);
            }
            // if( p >= p_max )
            // {
            //     p_max = p;
            // }
            // ROS_INFO("Data Length is %d", p);
            // ROS_INFO("Max Data Length is %d", p_max);
        }
        ros_ser.flush();

        loop_rate.sleep();
    }

}


