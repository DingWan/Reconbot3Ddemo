/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, IGM-RWTH Aachen University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 /**
  * \author Jorge De La Cruz
  * \version 0.1
  * \date February 20, 2017
  */


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "reconbot_control/EnableTorque.h"
#include "ReConBot.h"

//void reconbotCallback(control_msgs::FollowJointTrajectoryGoal goal);

void reconbotCallback(control_msgs::FollowJointTrajectoryGoal goal){
  ROS_INFO("=========================================================================");
  ROS_INFO("=========== Welcome to IGM - ReConBot Move Group Interface ==============");
  ROS_INFO("===========    Group of Robotic and Mechatronic            ==============");
  ROS_INFO("===========                                                ==============");
  ROS_INFO("===========                                                ==============");
  ROS_INFO("=========== Building trajectory ...                        ==============");
  ROS_INFO("=========================================================================");


  int i;
  float mode;
  float goalSize;
  ros::Duration time_rel;
  float mode_0;
  float mode_1;
  int init_index;
  int current_index;
  ros::ServiceClient client;
  ros::NodeHandle nh_Client;
  client = nh_Client.serviceClient<reconbot_control::EnableTorque>("enable_torque");
  control_msgs::FollowJointTrajectoryGoal goalMode;

  ReConBotLx RobotMode1;
  ReConBotLx RobotMode2;
  ReConBotLx RobotMode3;
  ReConBotLx RobotMode4;
  ReConBotLx RobotMode5;
  ReConBotLx RobotMode6;

  RobotMode1.nameSpace = "reconbot_controller";
  RobotMode2.nameSpace = "RA2C2_reconbot_controller";
  RobotMode3.nameSpace = "T2R5_reconbot_controller";
  RobotMode4.nameSpace = "T1R_reconbot_controller";
  RobotMode5.nameSpace = "T2R_reconbot_controller";
  RobotMode6.nameSpace = "T3R_reconbot_controller";

  std::string actMsg1;
  std::string actMsg2;
  std::string actMsg3;
  std::string actMsg4;
  std::string actMsg5;
  std::string actMsg6;

  actMsg1 = "Mode 1 activated";
  actMsg2 = "Mode 2 activated";
  actMsg3 = "Mode 3 activated";
  actMsg4 = "Mode 4 activated";
  actMsg5 = "Mode 5 activated";
  actMsg6 = "Mode 6 activated";


  RobotMode1.trajClient();
  RobotMode2.trajClient();
  RobotMode3.trajClient();
  RobotMode4.trajClient();
  RobotMode5.trajClient();
  RobotMode6.trajClient();

  reconbot_control::EnableTorque srv;

  goalSize = goal.trajectory.points.size();
  mode_0 = 1;
  init_index = 0;
  time_rel = ros::Duration(0);
  if (goalSize>1) {
    for (size_t i = 0; i < goalSize-1; i++) {
      mode_0 = goal.trajectory.points[i].positions[6];
      mode_1 = goal.trajectory.points[i+1].positions[6];

      if (mode_0!=mode_1) {
        current_index = i;
        if (mode_0 == 1) {
          goalMode = RobotMode1.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode1.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg1.c_str());
            RobotMode1.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }

        }

        if (mode_0 == 2) {
          goalMode = RobotMode2.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode2.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg2.c_str());
            RobotMode2.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 3) {
          goalMode = RobotMode3.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode3.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg3.c_str());
            RobotMode3.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 4) {
          goalMode = RobotMode4.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode4.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg4.c_str());
            RobotMode4.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 5) {
          goalMode = RobotMode5.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode5.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg5.c_str());
            RobotMode5.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 6) {
          goalMode = RobotMode6.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode6.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg6.c_str());
            RobotMode6.sendTrajectory(goalMode);
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }
      }
    }
  }
  if(goalSize==1){
    mode_0 = goal.trajectory.points[0].positions[6];
    if (mode_0 == 1) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode1.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg1.c_str());
        RobotMode1.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 2) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode2.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg2.c_str());
        RobotMode2.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 3) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode3.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg3.c_str());
        RobotMode3.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 4) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode4.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg4.c_str());
        RobotMode4.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 5) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode5.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg5.c_str());
        RobotMode5.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 6) {
      init_index = 0;
      current_index = 0;
      time_rel = ros::Duration(0);
      goalMode = RobotMode6.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg6.c_str());
        RobotMode6.sendTrajectory(goalMode);
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }
  }

  }



int main(int argc, char** argv)
{
  ROS_INFO("=========================================================================");
  ROS_INFO("=========== Welcome to IGM - ReConBot Move Group Interface ==============");
  ROS_INFO("===========    Group of Robotic and Mechatronic            ==============");
  ROS_INFO("===========                                                ==============");
  ROS_INFO("===========                                                ==============");
  ROS_INFO("=========== Waiting for a trajectory to execute...         ==============");
  ROS_INFO("=========================================================================");
  // Init the ROS node
  ros::init(argc, argv, "ReConBot_Driver");
  ros::NodeHandle nh_;
  ros::Subscriber sub_path;
  //ros::ServiceClient client;
  //ros::ServiceClient *clientPtr;
  //ros::waitForShutdown;
  //client = nh_.serviceClient<reconbot_control::EnableTorque>("enable_torque");
  //clientPtr = &client;
  sub_path = nh_.subscribe("/reconbot_trajectory", 100, reconbotCallback);

  ros::AsyncSpinner spinner(2);/**Two spinner are instantiated for managing 2 threats*/
  spinner.start();
  ros::waitForShutdown;

  // Start the trajectory
  // Wait for trajectory completion
  //while(!robot.getState().isDone() && ros::ok())
  //{
  //  usleep(50000);
  //}
  //sleep(8);
  ros::spin();
}
