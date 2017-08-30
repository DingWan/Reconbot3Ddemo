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
#include <pwd.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <dynamixel_msgs/MotorState.h>


using namespace std;

static ofstream torqueFile1;
static ofstream torqueFile2;
static ofstream torqueFile3;
static ofstream torqueFile4;
static ofstream torqueFile5;
static ofstream torqueFile6;



/**\fn void chatterCallback(control_msgs::FollowJointTrajectoryFeedback trajectory_point)
  * This callback function writes, into ~/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/encoders_data.txt
  *  file, the kinematics variables read for the encoders of the Dynamixel motors. This file contains the desired
  * trajectory point, which is a message of the type trajectory_msgs/JointTrajectoryPoint as well as the actual
  * trajectory point. These messages are saved in the variables desired_data and actual_data.
  *
  * The "sensor_data_capture" node starts recording the sensor data published in the "/RCB_full_mode_controller/state"
  *  topic as soon as the node is deployed and stops when is killed.
  **/

void chatterCallback(dynamixel_msgs::MotorStateList dynamixelList)
{
    dynamixel_msgs::MotorState dynamixel;
    dynamixel = dynamixelList.motor_states[0];
    float torque_ratio = dynamixel.load; /**< variable which contains the actual kinematic values*/
    int temperature = dynamixel.temperature;/**< variable which contains the desired kinematic values*/
    int motor_id = dynamixel.id;
    float time_stamp = dynamixel.timestamp;

    if (motor_id == 1){
        torqueFile1<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
    if (motor_id == 2){
        torqueFile2<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
    if (motor_id == 3){
        torqueFile3<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
    if (motor_id == 4){
        torqueFile4<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
    if (motor_id == 5){
        torqueFile5<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
    if (motor_id == 6){
        torqueFile6<<time_stamp <<"\t"<<torque_ratio<<"\t"<<temperature<<endl;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_data_capture");
  ros::NodeHandle n;

  passwd* pw = getpwuid(getuid());
  std::string path(pw->pw_dir);
  std::string sourcefile1;
  std::string sourcefile2;
  std::string sourcefile3;
  std::string sourcefile4;
  std::string sourcefile5;
  std::string sourcefile6;

  sourcefile1 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_1.txt";
  sourcefile2 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_2.txt";
  sourcefile3 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_3.txt";
  sourcefile4 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_4.txt";
  sourcefile5 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_5.txt";
  sourcefile6 = path += "/catkin_ws/src/reconbot/02_MATLAB_Code/sensors_data/torque_data_joint_6.txt";

  torqueFile1.open(sourcefile1.c_str());
  torqueFile2.open(sourcefile2.c_str());
  torqueFile3.open(sourcefile3.c_str());
  torqueFile4.open(sourcefile4.c_str());
  torqueFile5.open(sourcefile5.c_str());
  torqueFile6.open(sourcefile6.c_str());

  torqueFile1<<"time_stamp"<<"\t"<<"torque_joint_1"<<"\t"<<"temp_1"<<endl;
  torqueFile2<<"time_stamp"<<"\t"<<"torque_joint_2"<<"\t"<<"temp_2"<<endl;
  torqueFile3<<"time_stamp"<<"\t"<<"torque_joint_3"<<"\t"<<"temp_3"<<endl;
  torqueFile4<<"time_stamp"<<"\t"<<"torque_joint_4"<<"\t"<<"temp_4"<<endl;
  torqueFile5<<"time_stamp"<<"\t"<<"torque_joint_5"<<"\t"<<"temp_5"<<endl;
  torqueFile6<<"time_stamp"<<"\t"<<"torque_joint_6"<<"\t"<<"temp_6"<<endl;


  ros::Subscriber sub = n.subscribe("/motor_states/pan_tilt_port", 1000, chatterCallback);
  ros::spin();

  return 0;
}
