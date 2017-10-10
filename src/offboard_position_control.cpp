/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
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
#include <stdio.h>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <tf/tf.h>

//global variables declaration
ros::Time                   timer_end, timer_start;
geometry_msgs::Pose         real;
geometry_msgs::PoseArray    waypoints;
geometry_msgs::PoseStamped  goalPose;
mavros_msgs::State          currentState;
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
    ros::Subscriber stateSub = nh.subscribe<mavros_msgs::State> (uavNameSpace+"/mavros/state", 10, stateCb);
    ros::Subscriber localPoseSub = nh.subscribe(uavNameSpace+ "/mavros/local_position/pose", 1000, localPoseCallback);
    ros::Publisher localPosPub = nh.advertise<geometry_msgs::PoseStamped> (uavNameSpace+ "/mavros/setpoint_position/local", 10);
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool> (uavNameSpace+ "/mavros/cmd/arming");
    ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode> (uavNameSpace+ "/mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    //read setpoints from file
    std::string str1 = ros::package::getPath("kuri_mbzirc_offboard_control")+"/config/"+pathFile+ ".txt";
    const char * filename1 = str1.c_str();

    assert(filename1 != NULL);
    filename1 = strdup(filename1);
    FILE *file1 = fopen(filename1, "r");
    if (!file1)
    {
        std::cout<<"\nCan not open File";
        fclose(file1);
    }

    double locationx,locationy,locationz,qy;
    geometry_msgs::Pose pose;
    timer_start = ros::Time::now();

    int num = 0;
    while (!feof(file1))
    {
        fscanf(file1,"%lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qy);
        pose.position.x = locationx;
        pose.position.y = locationy;
        pose.position.z = locationz;
        tf::Quaternion tf_q ;
        tf_q= tf::createQuaternionFromYaw(qy);
        pose.orientation.x = tf_q.getX();
        pose.orientation.y = tf_q.getY();
        pose.orientation.z = tf_q.getZ();
        pose.orientation.w = tf_q.getW();

        //divide the path by checking the distance to the next point and comparing it to fixed distance
        if(num>0 && discFlag)
        {
            //copy of the last pushed waypoint (the waypoint before the next one)
            // waypoint[index] -----new1-----new2-----new3------pose
            int index = waypoints.poses.size()-1;
            geometry_msgs::Pose waypointPoseCopy;
            waypointPoseCopy.position.x = waypoints.poses[index].position.x;
            waypointPoseCopy.position.y = waypoints.poses[index].position.y;
            waypointPoseCopy.position.z = waypoints.poses[index].position.z;
            waypointPoseCopy.orientation.x = waypoints.poses[index].orientation.x;
            waypointPoseCopy.orientation.y = waypoints.poses[index].orientation.y;
            waypointPoseCopy.orientation.z = waypoints.poses[index].orientation.z;
            waypointPoseCopy.orientation.w = waypoints.poses[index].orientation.w;

            double dist = std::sqrt( ( (pose.position.x-waypointPoseCopy.position.x)*(pose.position.x-waypointPoseCopy.position.x) ) +
                                     ( (pose.position.y-waypointPoseCopy.position.y)*(pose.position.y-waypointPoseCopy.position.y) ) +
                                     ( (pose.position.z-waypointPoseCopy.position.z)*(pose.position.z-waypointPoseCopy.position.z) )
                                     );
            if(dist>distLimit)
            {
                std::cout<<" ******** waypoint descritization ******* "<<std::endl;
                int discretizeNum = dist/stepSize;
                double stepSizeCopy = stepSize;
                for(int i =0; i<discretizeNum; i++)
                {
                    double u = ( stepSize*(i+1) )/dist; //stepSize * (i+1) to find distance to the next waypoint during the descritization
                    geometry_msgs::Pose newPose;
                    //using parametric equations for finding the new point of x, y, z
                    newPose.position.x = ( (1-u)*waypointPoseCopy.position.x ) + (u*pose.position.x) ;
                    newPose.position.y = ( (1-u)*waypointPoseCopy.position.y ) + (u*pose.position.y) ;
                    newPose.position.z = ( (1-u)*waypointPoseCopy.position.z ) + (u*pose.position.z) ;
                    newPose.orientation.x = tf_q.getX();
                    newPose.orientation.y = tf_q.getY();
                    newPose.orientation.z = tf_q.getZ();
                    newPose.orientation.w = tf_q.getW();
                    waypoints.poses.push_back(newPose);
                    std::cout<<"new waypoints position: ("<<newPose.position.x<<", "<<newPose.position.y<<", "<<newPose.position.z<<") "<<std::endl;
                }

                //to add the exact waypoint since there is small floating points differences
                std::cout<<"waypoints position: ("<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z<<") "<<std::endl;
                std::cout<<"waypoints yaw: "<<qy<<" quetranion: "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<std::endl;
                waypoints.poses.push_back(pose);
            }
            else
            {
                std::cout<<"waypoints position: ("<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z<<") "<<std::endl;
                std::cout<<"waypoints yaw: "<<qy<<" quetranion: "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<std::endl;
                waypoints.poses.push_back(pose);
            }
        }
        else
        {
            std::cout<<"waypoints position: ("<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z<<") "<<std::endl;
            std::cout<<"waypoints yaw: "<<qy<<" quetranion: "<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<std::endl;
            waypoints.poses.push_back(pose);
        }

        num++;
    }

    //the initial goal
    goalPose.pose.position.x = waypoints.poses[count].position.x;
    goalPose.pose.position.y = waypoints.poses[count].position.y;
    goalPose.pose.position.z = waypoints.poses[count].position.z;
    goalPose.pose.orientation.x = waypoints.poses[count].orientation.x;
    goalPose.pose.orientation.y = waypoints.poses[count].orientation.y;
    goalPose.pose.orientation.z = waypoints.poses[count].orientation.z;
    goalPose.pose.orientation.w = waypoints.poses[count].orientation.w;

    //display for the first waypoint
    tf::Quaternion q1(goalPose.pose.orientation.x, goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w);
    double yaw1 = tf::getYaw(q1);
    std::cout<<"initial GOAL: "<<goalPose.pose.position.x<<" "<<goalPose.pose.position.y<<" "<<goalPose.pose.position.z<<std::endl;
    std::cout<<"goal ==> yaw: "<<yaw1<<" quetranion: "<<goalPose.pose.orientation.x<<" "<<goalPose.pose.orientation.y<<" "<<goalPose.pose.orientation.z<<" "<<goalPose.pose.orientation.w<<std::endl;
    std::cout<<"count : "<<count<<" waypoints num: "<<waypoints.poses.size()<<std::endl;

    //allAuto = true means arm switch offboard by the code otherwise you need to do using the RC
    if(allAuto)
    {
        offbSetMode.request.custom_mode = offbMode;
        armCmd.request.value = armFlag;
        lastRequest = ros::Time::now();
        statusUpdate = ros::Time::now();
    }


    while(ros::ok()){

        if (allAuto)
        {
            if( currentState.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if( setModeClient.call(offbSetMode) && offbSetMode.response.mode_sent){
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


        localPosPub.publish(goalPose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
