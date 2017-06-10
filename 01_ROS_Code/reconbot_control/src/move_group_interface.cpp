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

using namespace std;


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
  static ros::Duration time_rel;
  float mode_0;
  float mode_1;
  int init_index;
  int current_index;
  ros::ServiceClient client;
  ros::NodeHandle nh_Client;
  client = nh_Client.serviceClient<reconbot_control::EnableTorque>("enable_torque");
  control_msgs::FollowJointTrajectoryGoal goalMode;

  ReConBotLx RobotModeRef;
  ReConBotLx RobotMode0;
  ReConBotLx RobotMode1;
  ReConBotLx RobotMode2;
  ReConBotLx RobotMode3;
  ReConBotLx RobotMode4;
  ReConBotLx RobotMode5;
  ReConBotLx RobotMode6;
  ReConBotLx RobotMode7;
  ReConBotLx RobotMode8;
  ReConBotLx RobotMode9;
  ReConBotLx RobotMode10;
  ReConBotLx RobotMode11;
  ReConBotLx RobotMode12;


  RobotMode0.nameSpace = "RCB_full_mode_controller";
  RobotMode1.nameSpace = "RCB_3T2R_controller";
  RobotMode2.nameSpace = "RCB_3T1R_controller";
  RobotMode3.nameSpace = "RCB_1T1RA1C1_controller";
  RobotMode4.nameSpace = "RCB_1T1RA2C2_controller";
  RobotMode5.nameSpace = "RCB_1T1RA1C1A2C2_controller";
  RobotMode6.nameSpace = "RCB_2T2R6B_controller";
  RobotMode7.nameSpace = "RCB_2T2R6BX0Y0_controller";
  RobotMode8.nameSpace = "RCB_2T2R5B_controller";
  RobotMode9.nameSpace = "RCB_2T2R3B_controller";
  RobotMode10.nameSpace = "RCB_2RA1C1_controller";
  RobotMode11.nameSpace = "RCB_2RA2C2_controller";
  RobotMode12.nameSpace = "RCB_FIXED2U_controller";


  std::string actMsg0;
  std::string actMsg1;
  std::string actMsg2;
  std::string actMsg3;
  std::string actMsg4;
  std::string actMsg5;
  std::string actMsg6;
  std::string actMsg7;
  std::string actMsg8;
  std::string actMsg9;
  std::string actMsg10;
  std::string actMsg11;
  std::string actMsg12;


  actMsg0 = "Mode 0-9-12 activated";
  actMsg1 = "Mode 1-6-7 activated";
  actMsg2 = "Mode 2-3-4-5 activated";
  actMsg3 = "Mode 3 activated";
  actMsg4 = "Mode 4 activated";
  actMsg5 = "Mode 5 activated";
  actMsg6 = "Mode 6 activated";
  actMsg7 = "Mode 7 activated";
  actMsg8 = "Mode 8 activated";
  actMsg9 = "Mode 9 activated";
  actMsg10 = "Mode 10 activated";
  actMsg11 = "Mode 11 activated";
  actMsg12 = "Mode 12 activated";




  RobotMode0.trajClient();
  RobotMode1.trajClient();
  RobotMode2.trajClient();
  //RobotMode3.trajClient();
  //RobotMode4.trajClient();
  //RobotMode5.trajClient();
  //RobotMode6.trajClient();
  //RobotMode7.trajClient();
  RobotMode8.trajClient();
  //RobotMode9.trajClient();
  RobotMode10.trajClient();
  RobotMode11.trajClient();
  //RobotMode12.trajClient();

  reconbot_control::EnableTorque srv;

  goalSize = goal.trajectory.points.size();
  mode_0 = 1;
  init_index = 0;
  time_rel = ros::Duration(0.001);
  if (goalSize>1) {
    for (size_t i = 0; i < goalSize; i++) {
      mode_0 = goal.trajectory.points[i].positions[6];
      if (i<goalSize-1) {
        mode_1 = goal.trajectory.points[i+1].positions[6];
      }
      if (i==goalSize-1) {
        mode_1 = mode_0+1 ;
      }

      if (mode_0!=mode_1) {
        if (init_index==0) {
          current_index = i+1;
        }
        else{
          current_index = i;
        }
        if (mode_0 == 0 || mode_0 == 9 || mode_0 == 12) {
          goalMode = RobotMode0.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode0.new_time_rel;
          cout << "new time mode 0 =" << time_rel<<endl;
          cout << "init_index mode 2 =" << init_index<<endl;
          cout << "current_index mode 2 =" << current_index<<endl;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg0.c_str());
            RobotMode0.sendTrajectory(goalMode);
            RobotMode0.traj_client_->waitForResult();
          }
            else{
            ROS_INFO("The motors were not switched");
          }

        }

        if (mode_0 == 1 || mode_0 == 6 || mode_0 == 7) {
          goalMode = RobotMode1.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode1.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg1.c_str());
            RobotMode1.sendTrajectory(goalMode);
            RobotMode1.traj_client_->waitForResult();
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 2 || mode_0 == 3 || mode_0 == 4 || mode_0 == 5) {
          goalMode = RobotMode2.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode2.new_time_rel;
          cout << "new time mode 2 =" << time_rel<<endl;
          cout << "init_index mode 2 =" << init_index<<endl;
          cout << "current_index mode 2 =" << current_index<<endl;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg2.c_str());
            RobotMode2.sendTrajectory(goalMode);
            RobotMode2.traj_client_->waitForResult();
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 8) {
          goalMode = RobotMode8.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode8.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg8.c_str());
            RobotMode8.sendTrajectory(goalMode);
            RobotMode8.traj_client_->waitForResult();
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 10) {
          goalMode = RobotMode10.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode10.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg10.c_str());
            RobotMode10.sendTrajectory(goalMode);
            RobotMode10.traj_client_->waitForResult();
          }
          else{
            ROS_INFO("The motors were not switched");
          }
        }

        if (mode_0 == 11) {
          goalMode = RobotMode11.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
          time_rel = RobotMode11.new_time_rel;
          init_index = i;
          srv.request.motor_state = mode_0;
          if (client.call(srv)) {
            ROS_INFO("%s\n", actMsg11.c_str());
            RobotMode11.sendTrajectory(goalMode);
            RobotMode11.traj_client_->waitForResult();
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
    if (mode_0 == 0 || mode_0 == 9 || mode_0 == 12) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode0.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg0.c_str());
        RobotMode0.sendTrajectory(goalMode);
        RobotMode0.traj_client_->waitForResult();
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 1 || mode_0 == 6 || mode_0 == 7) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode1.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg1.c_str());
        RobotMode1.sendTrajectory(goalMode);
        RobotMode1.traj_client_->waitForResult();
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 2 || mode_0 == 3 || mode_0 == 4 || mode_0 == 5) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode2.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg2.c_str());
        RobotMode2.sendTrajectory(goalMode);
        RobotMode2.traj_client_->waitForResult();
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 8) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode8.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg8.c_str());
        RobotMode8.sendTrajectory(goalMode);
        RobotMode8.traj_client_->waitForResult();
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 10) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode10.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg10.c_str());
        RobotMode10.sendTrajectory(goalMode);
        RobotMode10.traj_client_->waitForResult();
      }
      else{
        ROS_INFO("The motors were not switched");
      }
    }

    if (mode_0 == 11) {
      init_index = 0;
      current_index = 1;
      time_rel = ros::Duration(0);
      goalMode = RobotMode11.getGoalMode(goal, time_rel, mode_0, init_index, current_index);
      srv.request.motor_state = mode_0;
      if (client.call(srv)) {
        ROS_INFO("%s\n", actMsg11.c_str());
        RobotMode11.sendTrajectory(goalMode);
        RobotMode11.traj_client_->waitForResult();
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
