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
  * \date August 29, 2017
  */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pwd.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>


using namespace std;

static ofstream torqueFile;
static dynamixel_msgs::MotorState dynamixel;


/**\fn void chatterCallback(dynamixel_msgs::MotorStateList dynamixelList)
  * This callback function writes, into
  * ~/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/encoders_data.txt
  *  file, the kinematics variables read for the encoders of the Dynamixel
  * motors. This file contains the desired
  * trajectory point, which is a message of the type
  * trajectory_msgs/JointTrajectoryPoint as well as the actual
  * trajectory point. These messages are saved in the variables desired_data
  * and actual_data.
  *
  * The "sensor_data_capture" node starts recording the sensor data published
  *in the "/RCB_full_mode_controller/state"
  *  topic as soon as the node is deployed and stops when is killed.
  **/

void chatterCallback(dynamixel_msgs::MotorStateList dynamixelList)
{
    std::vector<float> torque_ratio;
    std::vector<int> temperature;
    std::vector<float> time_stamp;

    torque_ratio.clear();
    temperature.clear();
    time_stamp.clear();

    for (size_t i = 0; i < 7; i++) {
      dynamixel = dynamixelList.motor_states[i];
      torque_ratio.push_back(dynamixel.load); /**< variable which contains the actual torque load, this is a ratio which is obtained dividing the actual torque by maximun Dynamixel motor's torque.*/
      temperature.push_back(dynamixel.temperature); /**< Working temperature of the motor in ºC. */
      time_stamp.push_back(dynamixel.timestamp);
    }
    torqueFile<<time_stamp[0]
          <<"\t"<<torque_ratio[0]<<"\t"<<torque_ratio[1]<<"\t"<<torque_ratio[2]
          <<"\t"<<torque_ratio[3]<<"\t"<<torque_ratio[4]<<"\t"<<torque_ratio[5]
          <<"\t"<<torque_ratio[6]
          <<"\t"<<temperature[0]<<"\t"<<temperature[1]<<"\t"<<temperature[2]
          <<"\t"<<temperature[3]<<"\t"<<temperature[4]<<"\t"<<temperature[5]
          <<"\t"<<temperature[6]<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_data_capture");
  ros::NodeHandle n;
  passwd* pw = getpwuid(getuid());
  std::string path(pw->pw_dir);
  std::string sourcefile;
  std::string topic;
  ros::Subscriber sub;

  sourcefile = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint.txt";

  torqueFile.open(sourcefile.c_str());
  torqueFile<<"time_stamp"
        <<"\t"<<"torque_joint_1"<<"\t"<<"torque_joint_2"<<"\t"<<"torque_joint_3"
        <<"\t"<<"torque_joint_4"<<"\t"<<"torque_joint_5"<<"\t"<<"torque_joint_6"
        <<"\t"<<"torque_joint_7"
        <<"\t"<<"temperature_1"<<"\t"<<"temperature_2"<<"\t"<<"temperature_3"
        <<"\t"<<"temperature_4"<<"\t"<<"temperature_5"<<"\t"<<"temperature_6"
        <<"\t"<<"temperature_7"<<endl;

  sub = n.subscribe("/motor_states/pan_tilt_port", 1000, chatterCallback);
  ros::spin();
  return 0;
}
