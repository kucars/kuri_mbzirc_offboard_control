/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "ar_pose/ar_single.h"
#include <iostream>
#include <stdio.h>

mavros_msgs::State currentState;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

visualization_msgs::Marker current_pose;
geometry_msgs::PoseStamped pose;
void pose_cb(const visualization_msgs::Marker::ConstPtr& msg2){
    current_pose = *msg2;
    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = -1*current_pose.pose.position.y;
    //pose.pose.position.z = current_pose.pose.position.z;
    //pose.pose.orientation.x = current_pose.pose.orientation.x;
    //pose.pose.orientation.y = current_pose.pose.orientation.y;
    //pose.pose.orientation.z = current_pose.pose.orientation.z;
    //pose.pose.orientation.w = current_pose.pose.orientation.w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, mavrosStateCallback);
    ros::Subscriber pose_sub = nh.subscribe<visualization_msgs::Marker>
            ("visualization_marker", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && currentState.connected){
        ros::spinOnce();
	// ros::spin();
        rate.sleep();
    }

    //printf ("%f \n",current_pose.pose.position.z);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 4;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    std::cout<<"I am hhere\n";
    while(ros::ok()){
        if( currentState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !currentState.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
