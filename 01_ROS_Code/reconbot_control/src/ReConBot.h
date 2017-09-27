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

#ifndef RECONBOT_HH
#define RECONBOT_HH

#include <pwd.h>
#include <sstream>
#include "reconbot_control/ModeState.h"
#include "reconbot_control/EnableTorque.h"


/** \class ReConBot
* \brief Class implemented for driving the ReConBot planning group.
* \details This class is created for controlling the Dynamixel Motors and also provides
* tools for taking control of the ReConBot using planning groups.
* Moveit.
*/

//typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

/**
 * Number of motors to take in account in the planning group. For instance, nMotors = 3 when planning
 * with right arm or the left arm of the ReConBot and nMotors = 6 when planning with bouth arms.
**/
static int nMotors;

/**
 * A vector with the numbers which identify the active motors. For example, for the
 * right arm should be motorActive = {1,2,3} because the motors with the ids 1 ,2 3 are
 * included in the kinematic chain of this arm.
**/
static int * motorsActive;
static std::vector<double> modesLx;
static   int number_of_lines;



/*****************************************************************************
** Main class
*****************************************************************************/

class ReConBot
{
protected:
  /**
   * Definition of the trajClient object used for publishing the desired trajectories in joint space.
  **/
  control_msgs::FollowJointTrajectoryGoal goal;
  control_msgs::FollowJointTrajectoryGoal goalMode;



  ros::NodeHandle nhPub;
  ros::NodeHandle nhSrv;
  ros::Publisher nhPub_pub;
  int topicQuery;
  std::vector<double> trajectoryPoints;

public:
  std::string sourceFile; /**< File directory with pose data */
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
  std::string nameSpace;
  std::vector<std::string> nameSpaces;
  std::vector<int> modes_sequence;
  std::vector<double> modes;
  float mod;
  std::string topicName; /**< Name given to the topic where is publishing.  */
  ReConBot(){
  }
  control_msgs::FollowJointTrajectoryGoal armTrajectory();
  bool run();
  //! Clean up the action client
  //control_msgs::JointTrajectoryControllerState getCurrentState(int nAm, std::string modeTopic);

};

/*****************************************************************************
** ReConBotLx class
*****************************************************************************/

class ReConBotLx : public ReConBot{
public:
  std::vector<double> modesLx;
  TrajClient* traj_client_;
  float pointsSize;
  ros::Duration new_time_rel;
  ReConBotLx(){
  }
  void trajClient();
  actionlib::SimpleClientGoalState getState();
  void sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  void motorsState(int arg[], int length);
  bool saveModesServer(reconbot_control::ModeState::Request& req, reconbot_control::ModeState::Response& res);
  control_msgs::FollowJointTrajectoryGoal getGoalMode(control_msgs::FollowJointTrajectoryGoal goal, ros::Duration time_rel, float mode, int init_index, int current_index);
};

/*****************************************************************************
** ReConBotPub class
*****************************************************************************/

class ReConBotPub : public ReConBotLx {
protected:
  int cont;
  int var;
  int j;
  float pos;

public:
  bool flag5;
  ReConBotPub(){
    sourceFile = "/home/jdelacruz/catkin_ws/src/reconbot/01_ROS_Code/trajectory/trajectory1.txt";
    topicName = "/reconbot_trajectory";
  }
  control_msgs::FollowJointTrajectoryGoal buildTrajectory();
  void trajectoryPublisherStart(ros::NodeHandle &nh, int topicQuery);
  void publisher(control_msgs::FollowJointTrajectoryGoal goal);
  //void mode();
};

/*************************************************************************
 ** Function definition
 ** All the class member functions have to be built or written below this
 ** seccion.
 *************************************************************************/

void ReConBotLx::trajClient(){
  /**
   * tell the action client that we want to spin a thread by default
   */
  traj_client_ = new TrajClient(nameSpace + "/follow_joint_trajectory", true);


  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(3.0))){
  ROS_INFO("Waiting for the joint_trajectory_action server");
  }
}

bool ReConBot::run(){
    ros::AsyncSpinner spinner(2);/**Two spinner are instantiated for managing 2 threats*/
    spinner.start();
    ros::waitForShutdown();
}

/**\fn void ReConBot::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  * Sends the command to start a given trajectory
  */
void ReConBotLx::sendTrajectory(control_msgs::FollowJointTrajectoryGoal goal){
  ROS_INFO("=========== Welcome to IGM - ReConBot Move Group Interface ==============");
  ROS_INFO("=========== Group of Robotic and Mechatronic               ==============");

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.001);
    traj_client_->sendGoal(goal);
    //traj_client_->waitForResult();
  }

/**
control_msgs::JointTrajectoryControllerState ReConBot::getCurrentState(int nAm, std::string modeTopic){
  int i;
  control_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(modeTopic);
  for (i=0; i<nAm i++) {
      currentState.actual.position.resize(nAm);
      currentState.actual.velocities.resize(nAm);
      currentState.actual.accelerations.resize(nAm);
      currentState.actual.positions[i] = state_msgs->actual.position[i];
      currentState.actual.velocities[i] = state_msgs->actual.velocities[i];
      currentState.actual.velocities[i] = state_msgs->actual.velocities[i];
      currentState.actual.accelerations[i] = state_msgs->actual.accelerations[i];
      currentState.header.stamp = ros::Time::now();
  }
  return currentState;
}
**/

void ReConBotLx::motorsState(int * arg, int length) {
  // First, the joint names, which apply to all waypoints
  motorsActive = arg;
  nMotors = length;
}


control_msgs::FollowJointTrajectoryGoal ReConBotLx::getGoalMode(control_msgs::FollowJointTrajectoryGoal goal, ros::Duration time_rel, float mode, int init_index, int current_index){
  pointsSize = current_index-init_index;
  int j = init_index;
  if (mode == 0 || mode == 9 || mode == 12) {
     for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_3");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_2");
      goalMode.trajectory.joint_names.push_back("joint_6");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(7);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[2];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[4];
      goalMode.trajectory.points[i].positions[5] = goal.trajectory.points[j].positions[5];
      goalMode.trajectory.points[i].positions[6] = goal.trajectory.points[j].positions[6];
      goalMode.trajectory.points[i].velocities.resize(7);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[2];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[4];
      goalMode.trajectory.points[i].velocities[5] = goal.trajectory.points[j].velocities[5];
      goalMode.trajectory.points[i].velocities[6] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(7);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[2];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[3];
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[4];
      goalMode.trajectory.points[i].accelerations[5] = goal.trajectory.points[j].accelerations[5];
      goalMode.trajectory.points[i].accelerations[6] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[j].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;

    }

  }

  if (mode == 1 || mode == 6 || mode == 7) {
    for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_3");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_6");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(6);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[2];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[5];
      goalMode.trajectory.points[i].positions[5] = goal.trajectory.points[j].positions[6];

      goalMode.trajectory.points[i].velocities.resize(6);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[2];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[5];
      goalMode.trajectory.points[i].velocities[5] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(6);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[2];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[3];;
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[5];
      goalMode.trajectory.points[i].accelerations[5] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[j].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;
    }
  }

  if (mode == 2 || mode == 3 || mode == 4 || mode == 5) {
    for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_2");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(5);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[4];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[6];
      goalMode.trajectory.points[i].velocities.resize(5);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[4];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(5);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[3];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[4];
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[j].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;
    }

  }

  if (mode == 8) {
    for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_3");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_2");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(6);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[2];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[4];
      goalMode.trajectory.points[i].positions[5] = goal.trajectory.points[j].positions[6];
      goalMode.trajectory.points[i].velocities.resize(6);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[2];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[4];
      goalMode.trajectory.points[i].velocities[5] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(6);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[2];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[3];
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[4];
      goalMode.trajectory.points[i].accelerations[5] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[i].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;
    }

  }

  if (mode == 10) {
    for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_3");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_2");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(6);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[2];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[4];
      goalMode.trajectory.points[i].positions[5] = goal.trajectory.points[j].positions[6];
      goalMode.trajectory.points[i].velocities.resize(6);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[2];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[4];
      goalMode.trajectory.points[i].velocities[5] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(6);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[2];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[3];
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[4];
      goalMode.trajectory.points[i].accelerations[5] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[j].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;
    }

  }

  if (mode == 11) {
    for (size_t i = 0; i < pointsSize; i++) {
      goalMode.trajectory.joint_names.push_back("joint_4");
      goalMode.trajectory.joint_names.push_back("joint_5");
      goalMode.trajectory.joint_names.push_back("joint_1");
      goalMode.trajectory.joint_names.push_back("joint_2");
      goalMode.trajectory.joint_names.push_back("joint_6");
      goalMode.trajectory.joint_names.push_back("joint_7");
      goalMode.trajectory.points.resize(pointsSize);
      goalMode.trajectory.points[i].positions.resize(6);
      goalMode.trajectory.points[i].positions[0] = goal.trajectory.points[j].positions[0];
      goalMode.trajectory.points[i].positions[1] = goal.trajectory.points[j].positions[1];
      goalMode.trajectory.points[i].positions[2] = goal.trajectory.points[j].positions[3];
      goalMode.trajectory.points[i].positions[3] = goal.trajectory.points[j].positions[4];
      goalMode.trajectory.points[i].positions[4] = goal.trajectory.points[j].positions[5];
      goalMode.trajectory.points[i].positions[5] = goal.trajectory.points[j].positions[6];
      goalMode.trajectory.points[i].velocities.resize(6);
      goalMode.trajectory.points[i].velocities[0] = goal.trajectory.points[j].velocities[0];
      goalMode.trajectory.points[i].velocities[1] = goal.trajectory.points[j].velocities[1];
      goalMode.trajectory.points[i].velocities[2] = goal.trajectory.points[j].velocities[3];
      goalMode.trajectory.points[i].velocities[3] = goal.trajectory.points[j].velocities[4];
      goalMode.trajectory.points[i].velocities[4] = goal.trajectory.points[j].velocities[5];
      goalMode.trajectory.points[i].velocities[5] = goal.trajectory.points[j].velocities[6];
      goalMode.trajectory.points[i].accelerations.resize(6);
      goalMode.trajectory.points[i].accelerations[0] = goal.trajectory.points[j].accelerations[0];
      goalMode.trajectory.points[i].accelerations[1] = goal.trajectory.points[j].accelerations[1];
      goalMode.trajectory.points[i].accelerations[2] = goal.trajectory.points[j].accelerations[3];
      goalMode.trajectory.points[i].accelerations[3] = goal.trajectory.points[j].accelerations[4];
      goalMode.trajectory.points[i].accelerations[4] = goal.trajectory.points[j].accelerations[5];
      goalMode.trajectory.points[i].accelerations[5] = goal.trajectory.points[j].accelerations[6];
      goalMode.trajectory.points[i].time_from_start = goal.trajectory.points[j].time_from_start - time_rel;
      if (i==pointsSize-1) {
        new_time_rel = goalMode.trajectory.points[i].time_from_start;
      }
      ++j;
    }

  }
  return goalMode;
}


//! Returns the current state of the action
actionlib::SimpleClientGoalState ReConBotLx::getState(){
  return traj_client_->getState();
}

void ReConBotPub::trajectoryPublisherStart(ros::NodeHandle &nh, int topicQuery){
  nhPub = nh;
  nhPub_pub = nhPub.advertise<control_msgs::FollowJointTrajectoryGoal>(topicName,topicQuery);
}

bool ReConBotLx::saveModesServer(reconbot_control::ModeState::Request& req, reconbot_control::ModeState::Response& res) {
  modesLx = req.mode_state;
  res.status = 1;
  return true;
}

void ReConBotPub::publisher(control_msgs::FollowJointTrajectoryGoal goal){
  int flag =0;
  int flag1 =0;
  while (ros::ok()&&flag==0) {
    if (flag1==0) {
      ROS_INFO("Waiting for Subscribers...");
    }
  //ros::Rate loop_rate(0.1);
  //nhPub_pub.publish(goal);
  //ros::spinOnce();
  //loop_rate.sleep();
  //ros::spinOnce();
  //ROS_INFO("Goal published");
//}
    if (nhPub_pub.getNumSubscribers()>= 1 && flag == 0) {
      nhPub_pub.publish(goal);
      ros::spinOnce();
      ROS_INFO("Goal published!");
      flag = 1;
    }
    flag1 =1;
  }
}
control_msgs::FollowJointTrajectoryGoal ReConBotPub::buildTrajectory(){
  ROS_INFO("Start building the Trajectory Vector...");
  for (size_t i = 0; i < nMotors; i++) {
    std::stringstream ss;
    ss << motorsActive[i];
    goal.trajectory.joint_names.push_back("joint_"+ ss.str());
  }
  goal.trajectory.joint_names.push_back("mode");


  std::fstream pathfile(sourceFile.c_str(), std::ios_base::in);
  number_of_lines = 0;
  std::string line;
  std::ifstream myfile(sourceFile.c_str());

  while (std::getline(myfile, line)){
      ++number_of_lines;
    }
  goal.trajectory.points.resize(number_of_lines);
  cont =0;
  var =0;
  int ref = 6;
  while (pathfile >> pos && ros::ok()) {
    trajectoryPoints.resize(number_of_lines*(nMotors*3+1));
    trajectoryPoints[var]=pos;
    ++var;
    if (var == nMotors*3 + 2) {
      j = 0;
      for (size_t i = 0; i < nMotors*3; i=i+3) {
        goal.trajectory.points[cont].positions.resize(nMotors+1);
        goal.trajectory.points[cont].positions[j] = trajectoryPoints[i];
        if (i==nMotors*3-3) {
          goal.trajectory.points[cont].positions[j+1] = trajectoryPoints[nMotors*3+1];
        }
        ++j;
      }
      j = 0;
      for (size_t i = 1; i < nMotors*3; i=i+3) {
        goal.trajectory.points[cont].velocities.resize(nMotors);
        goal.trajectory.points[cont].velocities[j] = trajectoryPoints[i];
        ++j;/* condition */
      }
      j = 0;
      for (size_t i = 2; i < nMotors*3; i=i+3) {
        goal.trajectory.points[cont].accelerations.resize(nMotors);
        goal.trajectory.points[cont].accelerations[j] = trajectoryPoints[i];
        if (i == nMotors*3 - 1) {
          goal.trajectory.points[cont].time_from_start = ros::Duration(trajectoryPoints[i+1]);
        }
        ++j;
      }
      var = 0;
      ++cont;
      ROS_INFO("New trajectory point created...");
    }
  }
  ROS_INFO("Trajectory vector was successfully built....");
  return goal;
}
/**
 * \todo Continue adding comments to the Class
 */

#endif
