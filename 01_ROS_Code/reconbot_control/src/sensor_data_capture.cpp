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
  * \email delacruz@igm.rwth-aachen.de
  * \date August 25, 2017
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <string>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

using namespace std;

static trajectory_msgs::JointTrajectoryPoint actual_data;
static trajectory_msgs::JointTrajectoryPoint desired_data;

static int i;
static ofstream trajectoryFile;


/**\fn void chatterCallback(control_msgs::FollowJointTrajectoryFeedback trajectory_point)
  * This callback function writes, into ~/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/encoders_data.txt
  *  file, the kinematics variables read for the encoders of the Dynamixel motors. This file contains the desired
  * trajectory point, which is a message of the type trajectory_msgs/JointTrajectoryPoint as well as the actual
  * trajectory point. These messages are saved in the variables desired_data and actual_data.
  *
  * The "sensor_data_capture" node starts recording the sensor data published in the "/RCB_full_mode_controller/state"
  *  topic as soon as the node is deployed and stops when is killed.
  **/

void chatterCallback(control_msgs::FollowJointTrajectoryFeedback trajectory_point)
{
    actual_data = trajectory_point.actual; /**< variable which contains the actual kinematic values*/
    desired_data = trajectory_point.desired; /**< variable which contains the desired kinematic values*/

    /** When the user is not requesting a desired trajectory which contains accelerations, the system by default
     * set the acceleration vector as a 0 or 1 size vector, therefore the size of the accelerations vector must be
     * manually resized in order to avoid any exception during compilation time.
     **/
    if (actual_data.accelerations.size() < 2){
        actual_data.accelerations.resize(6);
        actual_data.accelerations[0] = 0;
        actual_data.accelerations[1] = 0;
        actual_data.accelerations[2] = 0;
        actual_data.accelerations[3] = 0;
        actual_data.accelerations[4] = 0;
        actual_data.accelerations[5] = 0;
        desired_data.accelerations.resize(6);
        desired_data.accelerations[0] = 0;
        desired_data.accelerations[1] = 0;
        desired_data.accelerations[2] = 0;
        desired_data.accelerations[3] = 0;
        desired_data.accelerations[4] = 0;
        desired_data.accelerations[5] = 0;
    }

        trajectoryFile<<trajectory_point.header.stamp
                      <<"\t"<<actual_data.positions[0]
                      <<"\t"<<actual_data.positions[1]
                      <<"\t"<<actual_data.positions[2]
                      <<"\t"<<actual_data.positions[3]
                      <<"\t"<<actual_data.positions[4]
                      <<"\t"<<actual_data.positions[5]
                      <<"\t"<<actual_data.velocities[0]
                      <<"\t"<<actual_data.velocities[1]
                      <<"\t"<<actual_data.velocities[2]
                      <<"\t"<<actual_data.velocities[3]
                      <<"\t"<<actual_data.velocities[4]
                      <<"\t"<<actual_data.velocities[5]
                      <<"\t"<<actual_data.accelerations[0]
                      <<"\t"<<actual_data.accelerations[1]
                      <<"\t"<<actual_data.accelerations[2]
                      <<"\t"<<actual_data.accelerations[3]
                      <<"\t"<<actual_data.accelerations[4]
                      <<"\t"<<actual_data.accelerations[5]
                      <<"\t"<<actual_data.time_from_start
                      <<"\t"<<desired_data.positions[0]
                      <<"\t"<<desired_data.positions[1]
                      <<"\t"<<desired_data.positions[2]
                      <<"\t"<<desired_data.positions[3]
                      <<"\t"<<desired_data.positions[4]
                      <<"\t"<<desired_data.positions[5]
                      <<"\t"<<desired_data.velocities[0]
                      <<"\t"<<desired_data.velocities[1]
                      <<"\t"<<desired_data.velocities[2]
                      <<"\t"<<desired_data.velocities[3]
                      <<"\t"<<desired_data.velocities[4]
                      <<"\t"<<desired_data.velocities[5]
                      <<"\t"<<desired_data.accelerations[0]
                      <<"\t"<<desired_data.accelerations[1]
                      <<"\t"<<desired_data.accelerations[2]
                      <<"\t"<<desired_data.accelerations[3]
                      <<"\t"<<desired_data.accelerations[4]
                      <<"\t"<<desired_data.accelerations[5]
                      <<"\t"<<desired_data.time_from_start<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_data_capture");
  ros::NodeHandle n;
  trajectoryFile.open("~/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/encoders_data.txt");
  trajectoryFile<<"time_stamp"
               <<"\t"<<"actual_pos_joint_4"<<"\t"<<"actual_pos_joint_5"<<"\t"<<"actual_pos_joint_3"
               <<"\t"<<"actual_pos_joint_1"<<"\t"<<"actual_pos_joint_2"<<"\t"<<"actual_pos_joint_6"
               <<"\t"<<"actual_vel_joint_4"<<"\t"<<"actual_vel_joint_5"<<"\t"<<"actual_vel_joint_3"
               <<"\t"<<"actual_vel_joint_1"<<"\t"<<"actual_vel_joint_2"<<"\t"<<"actual_vel_joint_6"
               <<"\t"<<"actual_acc_joint_4"<<"\t"<<"actual_acc_joint_5"<<"\t"<<"actual_acc_joint_3"
               <<"\t"<<"actual_acc_joint_1"<<"\t"<<"actual_acc_joint_2"<<"\t"<<"actual_acc_joint_6"<<"\t"<<"actual_time_from_start"
              <<"\t"<<"desired_pos_joint_4"<<"\t"<<"desired_pos_joint_5"<<"\t"<<"desired_pos_joint_3"
              <<"\t"<<"desired_pos_joint_1"<<"\t"<<"desired_pos_joint_2"<<"\t"<<"desired_pos_joint_6"
              <<"\t"<<"desired_vel_joint_4"<<"\t"<<"desired_vel_joint_5"<<"\t"<<"desired_vel_joint_3"
              <<"\t"<<"desired_vel_joint_1"<<"\t"<<"desired_vel_joint_2"<<"\t"<<"desired_vel_joint_6"
              <<"\t"<<"desired_acc_joint_4"<<"\t"<<"desired_acc_joint_5"<<"\t"<<"desired_acc_joint_3"
              <<"\t"<<"desired_acc_joint_1"<<"\t"<<"desired_acc_joint_2"<<"\t"<<"desired_acc_joint_6"<<"\t"<<"desired_time_from_start"<<endl;

  ros::Subscriber sub = n.subscribe("/RCB_full_mode_controller/state", 1000, chatterCallback);
  ros::spin();

  return 0;
}
