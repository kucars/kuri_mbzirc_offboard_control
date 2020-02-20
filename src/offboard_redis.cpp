 
/***************************************************************************
 *   Copyright (C) 2019        by                                          *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <tf/tf.h>

#include <ctype.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>     // std::cout, std::ios
#include <sstream>      // std::ostringstream
#include "gripper/Attach.h"

//global variables declaration
ros::Time                   timer_end, timer_start;
geometry_msgs::Pose         real;
geometry_msgs::PoseArray    waypoints;
geometry_msgs::PoseStamped  goalPose;
mavros_msgs::State          currentState;
mavros_msgs::BatteryStatus  batteryState;
std::string                 uavNameSpace;
ros::Time                   lastRequest;
ros::Time                   statusUpdate;
mavros_msgs::SetMode        offbSetMode;
mavros_msgs::CommandBool    armCmd;
geometry_msgs::Pose 	    temp;

//variables defined by the user in the yaml file
float                       tolerance;
float                       toleranceO;
float                       distLimit;
std::string                 offbMode;
bool                        armFlag;
std::string                 pathFile;
bool                        allAuto;
bool                        discFlag;
int                         uavID;
float                       stepSize;

//variables used in the code
int count       = 0 ;
float errorX    = 0;
float errorY    = 0;
float errorZ    = 0;
float errorOX    = 0;
float errorOY    = 0;
float errorOZ    = 0;
float errorOW    = 0;
float errorOYaw    = 0;
double distance = 0;



std::string GetUAVCommand(std::string arg_cacheCmd)
{
    std::string del = "(";
    size_t pos = arg_cacheCmd.find(del);

    return arg_cacheCmd.substr(0, pos);
}

std::vector<float> GetCmdArguments(std::string arg_cacheCmd)
{
    // Note: Redis client does not allow spaces between arguments
    std::vector<float> argvector;

    std::string del1 = "(";
    std::string del2 = ")";
    std::string del3 = ",";

    size_t pos1 = arg_cacheCmd.find(del1);
    size_t pos2 = arg_cacheCmd.find(del2);

    std::string args = arg_cacheCmd.substr(pos1 + 1, (pos2 - pos1) - 1);

    // Check if there are arguments
    if (args.size() > 0)
    {
        size_t pos3 = 0;

        while ((pos3 = args.find(del3)) != std::string::npos)
        {
            // Extract the argument, convert float and add to the vector
            argvector.push_back(strtof((args.substr(0, pos3)).c_str(), 0));
            args.erase(0, pos3 + del3.length());
        }
        // Add the last argument
        argvector.push_back(strtof((args).c_str(), 0));
    }
    else {


    }

    return argvector;
}

bool isSDspace(char c)
{
    return((c == ' ') || (c == '\t') || (c == '\n') || (c == '\f') || (c == '\r') || (c == '\v'));
}



void localPoseCallback(const geometry_msgs :: PoseStamped :: ConstPtr& msg)
{
    real.position.x=msg ->pose.position.x;
    real.position.y=msg ->pose.position.y;
    real.position.z=msg ->pose.position.z;
    real.orientation.x=msg ->pose.orientation.x;
    real.orientation.y=msg ->pose.orientation.y;
    real.orientation.z=msg ->pose.orientation.z;
    real.orientation.w=msg ->pose.orientation.w;

    errorX =  goalPose.pose.position.x - real.position.x;
    errorY =  goalPose.pose.position.y - real.position.y;
    errorZ =  goalPose.pose.position.z - real.position.z;
    errorOX =  goalPose.pose.orientation.x - real.orientation.x;
    errorOY =  goalPose.pose.orientation.y - real.orientation.y;
    errorOZ =  goalPose.pose.orientation.z - real.orientation.z;
    errorOW =  goalPose.pose.orientation.w - real.orientation.w;

    tf::Quaternion q(real.orientation.x, real.orientation.y, real.orientation.z, real.orientation.w);
    double yaw = tf::getYaw(q);
    //    std::cout<<"real ==> yaw: "<<yaw<<" quetranion: "<<real.orientation.x<<" "<<real.orientation.y<<" "<<real.orientation.z<<" "<<real.orientation.w<<std::endl;
    tf::Quaternion q1(goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w);
    double yaw1 = tf::getYaw(q1);
    //    std::cout<<"goal ==> yaw: "<<yaw1<<" quetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<std::endl;

    errorOYaw = yaw1 - yaw;
    //    std::cout<<"\n\nyaw ERROR: "<<errorOYaw <<"\n\n"<<std::endl;
    //    std::cout<<"error: "<<errorX<<" "<<errorY<<" "<<errorZ<<" "<<errorOX<<" "<<errorOY<<" "<<errorOZ<<" "<<errorOW<<std::endl;
    //    std::cout<<"error: "<<errorX<<" "<<errorY<<" "<<errorZ<<" "<<errorOX<<" "<<errorOY<<" "<<errorOZ<<" "<<errorOW<<std::endl;
    if(count == 0)
    {
        temp.position.x = msg ->pose.position.x;
        temp.position.y = msg ->pose.position.y;
        temp.position.z = msg ->pose.position.z;
        temp.orientation.x = msg ->pose.orientation.x;
        temp.orientation.y = msg ->pose.orientation.y;
        temp.orientation.z = msg ->pose.orientation.z;
        temp.orientation.w = msg ->pose.orientation.w;
    }else
    {
        double dist = std::sqrt( ( (msg ->pose.position.x-temp.position.x)*(msg ->pose.position.x-temp.position.x) ) +
                                 ( (msg ->pose.position.y-temp.position.y)*(msg ->pose.position.y-temp.position.y) ) +
                                 ( (msg ->pose.position.z-temp.position.z)*(msg ->pose.position.z-temp.position.z) )
                                 );
        distance +=dist;
        temp.position.x = msg ->pose.position.x;
        temp.position.y = msg ->pose.position.y;
        temp.position.z = msg ->pose.position.z;
        temp.orientation.x = msg ->pose.orientation.x;
        temp.orientation.y = msg ->pose.orientation.y;
        temp.orientation.z = msg ->pose.orientation.z;
        temp.orientation.w = msg ->pose.orientation.w;

    }
    if ((fabs(errorX) < tolerance) && (fabs(errorY) < tolerance) && (fabs(errorZ) < tolerance)
            && (fabs(errorOYaw) <toleranceO)
            //            && (fabs(errorOX) < toleranceO) && (fabs(errorOY) < toleranceO) && (fabs(errorOZ) < toleranceO) && (fabs(errorOW) < toleranceO)
            )
    {
        count++;
        if(count<waypoints.poses.size())
        {

            goalPose.pose.position.x = waypoints.poses[count].position.x;
            goalPose.pose.position.y = waypoints.poses[count].position.y;
            goalPose.pose.position.z = waypoints.poses[count].position.z;
            goalPose.pose.orientation.x = waypoints.poses[count].orientation.x;
            goalPose.pose.orientation.y = waypoints.poses[count].orientation.y;
            goalPose.pose.orientation.z = waypoints.poses[count].orientation.z;
            goalPose.pose.orientation.w = waypoints.poses[count].orientation.w;

            //display the next goal
            std::cout<<"new GOAL: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<std::endl;
            std::cout<<"real ==> yaw: "<<yaw<<" quetranion: "<<real.orientation.x<<" "<<real.orientation.y<<" "<<real.orientation.z<<" "<<real.orientation.w<<std::endl;
            tf::Quaternion q2(goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w);
            double yaw2 = tf::getYaw(q2);
            std::cout<<"goal ==> yaw: "<<yaw2<<" quetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<std::endl;
            std::cout<<"count : "<<count<<" waypoints num: "<<waypoints.poses.size()<<std::endl<<std::endl;
        }
        if(count == waypoints.poses.size())
        {
            std::cout<<">>>>> Finished :) <<<<<"<<std::endl;
            timer_end = ros::Time::now();
            std::cout<<"\nPath execution took:"<<double(timer_end.toSec() - timer_start.toSec())<<" secs"<<std::endl;
            std::cout<<"\nPath distance :"<<distance<<" m"<<std::endl;

        }

    }

}

void stateCb(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

void battCb(const mavros_msgs::BatteryStatus::ConstPtr& msg){
    batteryState = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_position");
    ros::NodeHandle nh;

    //initialize params
    ros::param::param<int>("~uav_id", uavID, -1); //-1 means no id, no namespace
    ros::param::param<std::string>("~path_file", pathFile, "real_test3");
    ros::param::param<float>("~distance_limit", distLimit, 0.2);
    ros::param::param<float>("~step_size", stepSize, 0.2);
    ros::param::param<bool>("~all_auto", allAuto, false);
    ros::param::param<std::string>("~offboard_set_mode", offbMode, "OFFBOARD");
    ros::param::param<bool>("~arm_command", armFlag, true);
    ros::param::param<float>("~tolerance_position", tolerance, 0.15);
    ros::param::param<float>("~tolerance_yaw", toleranceO, 0.2);
    ros::param::param<bool>("~discretize", discFlag, false);

    //display the paramter
    std::cout<<" ******** Offboard Paramters ******* "<<std::endl;
    std::cout<<"uav_id : \t\t"<<uavID <<std::endl;
    std::cout<<"path_file: \t\t"<<pathFile <<std::endl;
    std::cout<<"step_size: \t\t"<< stepSize<<std::endl;
    std::cout<<"all_auto: \t\t"<<allAuto <<std::endl;
    std::cout<<"offboard_set_mode: \t"<<offbMode <<std::endl;
    std::cout<<"arm_command: \t\t"<<armFlag <<std::endl;
    std::cout<<"tolerance_position: \t"<<tolerance <<std::endl;
    std::cout<<"tolerance_yaw: \t\t"<<toleranceO <<std::endl;
    std::cout<<"discretize: \t\t"<<discFlag <<std::endl;
    std::cout<<" \n\n "<<std::endl;


    if(uavID != -1)
        uavNameSpace = "/uav_"+boost::lexical_cast<std::string>(uavID);
    else
        uavNameSpace = "";

    ros::Subscriber stateSub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 10, stateCb);
    ros::Subscriber battSub = nh.subscribe<mavros_msgs::BatteryStatus> ("/mavros/battery", 10, battCb);
    ros::Subscriber localPoseSub = nh.subscribe("/mavros/local_position/pose", 1000, localPoseCallback);
    ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped> ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
    ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

    ros::ServiceClient gripperClient = nh.serviceClient<gripper::Attach>("/GripperOnOFF");
    gripper::Attach srv;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //############################################################
    FILE *fp_flushDb;
    fp_flushDb = popen("redis-cli flushall", "r");

    pclose(fp_flushDb);
    fp_flushDb = NULL;


    // Setup OSDK.
    //    LinuxSetup linuxEnvironment(argc, argv);
    //    Vehicle*   vehicle = linuxEnvironment.getVehicle();
    //    if (vehicle == NULL)
    //    {
    //        std::cout << "Vehicle not initialized, exiting.\n";
    //        return -1;
    //    }

    //    // Obtain Control Authority
    //    vehicle->obtainCtrlAuthority(functionTimeout);


    //############################################################



 

    //allAuto = true means arm switch offboard by the code otherwise you need to do using the RC
    if(allAuto)
    {
        offbSetMode.request.custom_mode = offbMode;
        armCmd.request.value = armFlag;
        lastRequest = ros::Time::now();
        statusUpdate = ros::Time::now();
    }


    // ############## initial point ############
    tf::Quaternion tf_q ;
    tf_q= tf::createQuaternionFromYaw(0);
    goalPose.header.frame_id = "world";
    goalPose.header.stamp = ros::Time::now();
    goalPose.pose.position.x = -0.22;
    goalPose.pose.position.y = -0.4;
    goalPose.pose.position.z = 1.3;
    goalPose.pose.orientation.x = tf_q.getX();
    goalPose.pose.orientation.y = tf_q.getY();
    goalPose.pose.orientation.z = tf_q.getZ();
    goalPose.pose.orientation.w = tf_q.getW();

    FILE *fp;
    char var[40];
    while(ros::ok()){

        if (allAuto)
        {
            if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if( setModeClient.call(offbSetMode) && offbSetMode.response.success){
                    ROS_INFO("Offboard enabled");
                }
                lastRequest = ros::Time::now();
            } else {
                if( !currentState.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                    if( armingClient.call(armCmd) && armCmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    lastRequest = ros::Time::now();
                }
                if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
                {
                    // std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
                    statusUpdate = ros::Time::now();
                }
            }
        }


        //



        if ((fp = popen("redis-cli -h 10.0.0.151 GET uavcmd", "r")) != NULL)
        {
            fgets(var, sizeof(var), fp);

            std::string cacheCmd = var;

            if (cacheCmd != "NONE\n")
            {
                // Remove all spaces in the command
                cacheCmd.erase(remove_if(cacheCmd.begin(), cacheCmd.end(), isSDspace), cacheCmd.end());
                std::cout << cacheCmd << std::endl;

                std::string cmd = GetUAVCommand(cacheCmd);
                std::vector<float> cmdArgs = GetCmdArguments(cacheCmd);

                if (cmd == "arm") {
                    std::cout << "UAV armed\n" << std::endl;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;

                }

                if (cmd == "disarm") {
                    std::cout << "UAV disarmed\n" << std::endl;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;

                }

                if (cmd == "takeoff") {
                    std::cout << "UAV taking off...\n" << std::endl;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;


                }

                if (cmd == "land") {
                    std::cout << "UAV Landing...\n" << std::endl;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;


                }
                if (cmd == "get_nedcoords") {
                    std::cout << "getting currunt location..." << std::endl;
                    geometry_msgs::Pose pp;
                    pp.position = real.position;
                    pp.orientation = real.orientation;
                    tf::Quaternion q2(real.orientation.x, real.orientation.y, real.orientation.z, real.orientation.w);
                    double yaw2 = tf::getYaw(q2);

                    FILE *fp2;
                    std::ostringstream ss;
                    ss << pp.position.x<<","<<pp.position.y<<","<<pp.position.z<<","<<yaw2;
                    std::string word = "redis-cli -h 10.0.0.151 SET nedcoords "+ss.str();
//                    std::cout<<word<<"\n"<<std::endl;
                    const char* word_cc = word.c_str();
                    fp2 = popen(word_cc, "r");
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;


                }
                if (cmd == "get_energy") {
                    std::cout << "getting currunt energy level..." << std::endl;
                    double pp;
                    pp = batteryState.voltage;
                    FILE *fp2;
                    std::ostringstream ss;
                    ss << pp;
                    std::string word = "redis-cli -h 10.0.0.151 SET energy "+ss.str();
//                    std::cout<<word<<"\n"<<std::endl;
                    const char* word_cc = word.c_str();
                    fp2 = popen(word_cc, "r");
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;

                }
                if (cmd == "fly_to") {
                    std::cout << "MoveTo..." << std::endl;

                    if (cmdArgs.size() != 4) {
                        std::cout << "Cmd : fly_to - parameters incorrect..." << std::endl;
                    }
                    else {
                        float x = cmdArgs[0];
                        float y = cmdArgs[1];
                        float z = cmdArgs[2];
                        float v = cmdArgs[3];

                        //client.moveToPositionAsync(x, y, z, v);
                        //                    moveByPositionOffset(vehicle, x, y, z, v);

                        std::cout << "UAV flying..." << std::endl;

                        std::ostringstream sx, sy, sz, sv;
                        sx << x;
                        sy << y;
                        sz << z;
                        sv << v;
                        std::string respmsg("Target: x = " + sx.str() + " y = " + sy.str() + " z = " + sz.str());
                        std::cout << respmsg << std::endl;

                        std::string respmsg2("Yaw = " + sv.str() + "\n");
                        std::cout << respmsg2 << std::endl;
                        tf::Quaternion tf_q ;
                        tf_q= tf::createQuaternionFromYaw(v);
                        goalPose.pose.position.x = x;
                        goalPose.pose.position.y = y;
                        goalPose.pose.position.z = z;
                        goalPose.pose.orientation.x = tf_q.getX();
                        goalPose.pose.orientation.y = tf_q.getY();
                        goalPose.pose.orientation.z = tf_q.getZ();
                        goalPose.pose.orientation.w = tf_q.getW();
                        std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;


                    }
                    //Sleep(5000);
                }

                if (cmd == "rotate_to") {
                    std::cout << "MoveTo..." << std::endl;

                    if (cmdArgs.size() != 1) {
                        std::cout << "Cmd : rotate_to - parameters incorrect..." << std::endl;
                    }
                    else {
                        float w = cmdArgs[0];

                        //client.moveToPositionAsync(x, y, z, v);
                        //                    moveByPositionOffset(vehicle, x, y, z, v);

                        std::cout << "UAV rotating..." << std::endl;

                        std::ostringstream sw;
                        sw << w;

                        std::string respmsg2("Yaw = " + sw.str() + "\n");
                        std::cout << respmsg2 << std::endl;
                        tf::Quaternion tf_q ;
                        tf_q= tf::createQuaternionFromYaw(w);
                        goalPose.pose.orientation.x = tf_q.getX();
                        goalPose.pose.orientation.y = tf_q.getY();
                        goalPose.pose.orientation.z = tf_q.getZ();
                        goalPose.pose.orientation.w = tf_q.getW();
                        std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;


                    }
                    //Sleep(5000);
                }

                if (cmd == "hover") {
                    std::cout << "UAV hovering..." << std::endl;
                }

                if (cmd == "acquire_cargo")
                {
//                    goalPose.pose.position.x = 0.32;
//                    goalPose.pose.position.y = -0.36;

                    std::cout << "acquiring the cargo..." << std::endl;
                    goalPose.pose.position.z = 0.69;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;
//                     srv.request.OnOff = true;
//                     gripperClient.call(srv);

                }
                if (cmd == "release_cargo")
                {
//                    goalPose.pose.position.x = 0.32;
//                    goalPose.pose.position.y = -0.36;
//                    goalPose.pose.position.z = 0.65;
                    std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;
                    std::cout << "releasing the cargo..." << std::endl;
                    srv.request.OnOff = false;
                    gripperClient.call(srv);

                }
                //*var = 0; ?????
                FILE *fp2;

                if ((fp2 = popen("redis-cli -h 10.0.0.151 SET uavcmd NONE", "r")) != NULL)
                {
                    pclose(fp2);
                    fp2 = NULL;
                }

            }
            pclose(fp);
            fp = NULL;
        }

        localPosPub.publish(goalPose);
//        std::cout<<"\ngoal ==> position: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<" "<<"\nquetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<"\n"<<std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
