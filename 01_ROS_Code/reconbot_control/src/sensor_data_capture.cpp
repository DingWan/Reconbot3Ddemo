#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>
#include <string>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

using namespace std;

static trajectory_msgs::JointTrajectoryPoint actual_data;
static int i;
static ofstream trajectoryFile;


void chatterCallback(control_msgs::FollowJointTrajectoryFeedback trajectory_point)
{
    //ofstream trajectoryFile;
    //trajectoryFile.open("sensor_data.txt");
    actual_data = trajectory_point.actual;
    //trajectoryFile<<actual_data;
    //trajectorySize = trajectory.points.size();
    if (actual_data.accelerations.size() < 2){
        actual_data.accelerations.resize(6);
        actual_data.accelerations[0] = 0;
        actual_data.accelerations[1] = 0;
        actual_data.accelerations[2] = 0;
        actual_data.accelerations[3] = 0;
        actual_data.accelerations[4] = 0;
        actual_data.accelerations[5] = 0;
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
                      <<"\t"<<actual_data.time_from_start<<endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_data_capture");
  ros::NodeHandle n;
  i=0;
  trajectoryFile.open("sensor_data.txt");
  trajectoryFile<<"time_stamp"
               <<"\t"<<"pos_joint_4"<<"\t"<<"pos_joint_5"<<"\t"<<"pos_joint_3"
               <<"\t"<<"pos_joint_1"<<"\t"<<"pos_joint_2"<<"\t"<<"pos_joint_6"
               <<"\t"<<"vel_joint_4"<<"\t"<<"vel_joint_5"<<"\t"<<"vel_joint_3"
               <<"\t"<<"vel_joint_1"<<"\t"<<"vel_joint_2"<<"\t"<<"vel_joint_6"
               <<"\t"<<"acc_joint_4"<<"\t"<<"acc_joint_5"<<"\t"<<"acc_joint_3"
               <<"\t"<<"acc_joint_1"<<"\t"<<"acc_joint_2"<<"\t"<<"acc_joint_6"<<"\t"<<"time_from_start"<<endl;

  ros::Subscriber sub = n.subscribe("/RCB_full_mode_controller/state", 1000, chatterCallback);
  ros::spin();

  return 0;
}
