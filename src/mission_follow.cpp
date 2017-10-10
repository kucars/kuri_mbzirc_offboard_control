/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>

mavros_msgs::State currentState;
void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  currentState = *msg;
  std::cout<<"Current Mode is: "<<msg->mode<<"\n"; fflush(stdout);
  ROS_INFO("Sending Arming message FAILED!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission");
  ros::NodeHandle nh;

  ros::Subscriber statePub              = nh.subscribe<mavros_msgs::State>("/uav_2/mavros/state", 1, mavrosStateCallback);
  ros::ServiceClient armingClient       = nh.serviceClient<mavros_msgs::CommandBool>("/uav_2/mavros/cmd/arming");
  ros::ServiceClient setModeClient      = nh.serviceClient<mavros_msgs::SetMode>("/uav_2/mavros/set_mode");
  ros::ServiceClient setMissionWaypoint = nh.serviceClient<mavros_msgs::WaypointPush>("/uav_2/mavros/mission/push");

  ros::Rate rate(20.0);
  mavros_msgs::SetMode setMode;
  mavros_msgs::WaypointPush pushService;
  mavros_msgs::Waypoint waypoint;

  waypoint.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL;
  waypoint.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  waypoint.is_current   = false;
  waypoint.autocontinue = false;
  waypoint.x_lat        = 47.3977998;
  waypoint.y_long       = 8.5456246;
  waypoint.z_alt        = 10;

  pushService.request.waypoints.push_back(waypoint);
  if(setMissionWaypoint.call(pushService))
  {
    ROS_INFO("Waypoint Sent Responce %d",pushService.response.success);
  }
  else
  {
    ROS_ERROR("FAILED");
  }

  setMode.request.custom_mode = "AUTO.MISSION";
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time lastRequestT    = ros::Time::now();
  ros::Time statusUpdate    = ros::Time::now();

  while(ros::ok())
  {
    // This code overrides the RC mode and is dangerous when performing tests: re-use in real flight tests
    if( currentState.mode != "AUTO.MISSION" && (ros::Time::now() - lastRequestT > ros::Duration(5.0)))
    {
      if( setModeClient.call(setMode) && setMode.response.mode_sent)
      {
        ROS_INFO("AUTO enabled");
      }
      else
      {
        ROS_ERROR("Failed to change to AUTO mode");
      }
      lastRequestT = ros::Time::now();
    }

    //if( currentState.mode == "AUTO.RTL")
    {
      if( !currentState.armed && (ros::Time::now() - lastRequestT > ros::Duration(5.0)))
      {
        if( armingClient.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("ARMING Command send through mavros, check messages related to safety switch");
        }
        else
        {
          ROS_INFO("Sending Arming message FAILED!");
        }
        lastRequestT = ros::Time::now();
      }
    }
    if(ros::Time::now() - statusUpdate > ros::Duration(1.0))
    {
      std::cout<<"Current Mode is: "<<currentState.mode<<"\n"; fflush(stdout);
      statusUpdate = ros::Time::now();
    }
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
  return 0;
}
