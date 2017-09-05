/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, IGM-RWTH Aachen University
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
  * The ReConBot Package includes the dynamixel_controllers package for
  * the Dynamixel Motors (https://github.com/arebgun/dynamixel_motor) and
  * the reconbot_control package developed at IGM (RWTH Aachen University),
  * this package provides a main class called ReConBot.h that allows other program
  * to establish communication with the MX-64AT motors as well as planning
  * trajectories for each of the arms of the ReConBot or the whole group.
  * In addition, this package allows the user to establish communication with other
  * nodes, running in MATLAB for instance, throught specific topics.
  */


 /**
  * \mainpage
  * \author Jorge De La Cruz
  * \version 0.1
  * \date February 28, 2017
  */

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>
#include "ReConBot.h"
#include <string>
#include <vector>
#include <cstdlib>
#include <map>
#include <pwd.h>
using namespace std;


int main(int argc,char **argv){
  ros::init(argc, argv, "Trajectory_Publisher");
  ros::NodeHandle nh;
  control_msgs::FollowJointTrajectoryGoal goal;
  //ros::Rate loop_rate(5);
  //ros::AsyncSpinner spinner(1);/**<Two spinner are instantiated for managing 2 threats*/
  //spinner.start();
  ReConBotPub Publisher;
  Publisher.topicName = "/reconbot_trajectory";
  int arg[] = {4,5,3,1,2,6,7};

  passwd* pw = getpwuid(getuid());
  std::string path(pw->pw_dir);

  Publisher.motorsState(arg,7);
  Publisher.trajectoryPublisherStart(nh, 1000);
  Publisher.nameSpace = "reconbot_controller";
  Publisher.sourceFile = path += "/catkin_ws/src/reconbot/01_ROS_Code/trajectory/trajectory.txt";
  goal = Publisher.buildTrajectory();
  Publisher.publisher(goal);

  sleep(1);

  ROS_INFO("Good bye!");
  //spinner.stop();
  //loop_rate.sleep();

  //sleep(2);
  return 0;
}
