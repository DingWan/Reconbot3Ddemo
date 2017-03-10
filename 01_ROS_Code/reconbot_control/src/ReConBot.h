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
  * \version 1.0
  * \date February 20, 2017
  */

#ifndef RECONBOT_HH
#define RECONBOT_HH

#include <pwd.h>
#include <sstream>


/** \class ReConBot
* \brief Class implemented for driving the ReConBot planning group.
* \details This class is created for controlling the Dynamixel Motors and also provides
* tools for taking control of the ReConBot using planning groups.
* Moveit.
*/

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

/**
 * Number of motors to take in account in the planning group. For instance, nMotors = 3 when planning
 * with right arm or the left arm of the ReConBot and nMotors = 6 when planning with bouth arms.
**/
static int nMotors;

/**
 * A vector with the numbers which identify the active motors. For example, for the
 * right arm should be motorActive = {1,2,3} because the motors with the ids 1 ,2 3 are
 * included in the kinematic chain of thid arm.
**/
static int * motorsActive;
class ReConBot
{
protected:
  /**
   * Definition o fthe trajClient object used for publish the desired trajectories in joint space.
  **/
  TrajClient* traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal;
  ros::NodeHandle nhPub;
  ros::Publisher nhPub_pub;
  int topicQuery;
  std::vector<double> trajectoryPoints;

public:
  std::string sourceFile; /**< File directory with pose data */
  std::string nameSpace;
  std::string topicName; /**< Name given to the topic where is publishing.  */
  ReConBot(){
  }
  void trajClient();
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
  control_msgs::FollowJointTrajectoryGoal armTrajectory();
  bool run();
  actionlib::SimpleClientGoalState getState();
  //! Clean up the action client

};
class ReConBotLx : public ReConBot{
public:
  ReConBotLx(){
  }
  void motorsState(int arg[], int length);
  void reconbotCallback(const control_msgs::FollowJointTrajectoryGoal goal);
};

class ReConBotPub : public ReConBotLx {
protected:
  int cont;
  int var;
  int j;
  float pos;

public:
  ReConBotPub(){
    sourceFile = "/home/jdelacruz/catkin_ws/src/reconbot/01_ROS_Code/trajectory/trajectory1.txt";
    topicName = "/reconbot_trajectory";
  }
  control_msgs::FollowJointTrajectoryGoal buildTrajectory();
  void trajectoryPublisherStart(ros::NodeHandle &nh, int topicQuery);
  void publisher(control_msgs::FollowJointTrajectoryGoal goal);
  //void mode();
};

void ReConBot::trajClient(){
  /**
   * tell the action client that we want to spin a thread by default
   */
  traj_client_ = new TrajClient(nameSpace + "/follow_joint_trajectory", true);

  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
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
void ReConBot::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal){
  ROS_INFO("=========== Welcome to IGM - ReConBot Move Group Interface ==============");
  ROS_INFO("=========== Group of Robotic and Mechatronic               ==============");

    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

void ReConBotLx::motorsState(int * arg, int length) {
  // First, the joint names, which apply to all waypoints
  motorsActive = arg;
  nMotors = length;
}

void ReConBotLx::reconbotCallback(const control_msgs::FollowJointTrajectoryGoal goal){
      ReConBot::startTrajectory(goal);
  }

//! Returns the current state of the action
actionlib::SimpleClientGoalState ReConBot::getState(){
  return traj_client_->getState();
}

void ReConBotPub::trajectoryPublisherStart(ros::NodeHandle &nh, int topicQuery){
  nhPub = nh;
  nhPub_pub = nhPub.advertise<control_msgs::FollowJointTrajectoryGoal>(topicName,topicQuery);
}

void ReConBotPub::publisher(control_msgs::FollowJointTrajectoryGoal goal){
  int flag =0;
  while (ros::ok()&&flag==0) {
    ROS_INFO("Waiting for Subscribers...");
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
  }
}
control_msgs::FollowJointTrajectoryGoal ReConBotPub::buildTrajectory(){
  ROS_INFO("Start building the Trajectory Vector...");
  for (size_t i = 0; i < nMotors; i++) {
    std::stringstream ss;
    ss << motorsActive[i];
    goal.trajectory.joint_names.push_back("joint_"+ ss.str());
  }
  //goal.trajectory.joint_names.push_back("joint_1");
  //goal.trajectory.joint_names.push_back("joint_2");
  //goal.trajectory.joint_names.push_back("joint_3");

  std::fstream pathfile(sourceFile.c_str(), std::ios_base::in);
  int number_of_lines = 0;
  std::string line;
  std::ifstream myfile(sourceFile.c_str());

  while (std::getline(myfile, line)){
      ++number_of_lines;
    }
  goal.trajectory.points.resize(number_of_lines);
  cont =0;
  var =0;
  while (pathfile >> pos && ros::ok()) {
    trajectoryPoints.resize(number_of_lines*(nMotors*3+1));
    trajectoryPoints[var]=pos;
    ++var;
    if (var == nMotors*3 + 1) {
      j = 0;
      for (size_t i = 0; i < nMotors*3; i=i+3) {
        goal.trajectory.points[cont].positions.resize(nMotors);
        goal.trajectory.points[cont].positions[j] = trajectoryPoints[i];
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
