#include "ros/ros.h"
#include "std_msgs/String.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <fstream>


void chatterCallback(control_msgs::JointTrajectoryControllerState trajectory_point)
{
    actual_data = trajectory_point.actual
    //trajectorySize = trajectory.points.size();
    if (i==0){
    trajectoryFile<<"time_stamp"
                 <<"\t"<<"pos_joint_4"<<"\t"<<"pos_joint_5"<<"\t"<<"pos_joint_3"
                 <<"\t"<<"pos_joint_1"<<"\t"<<"pos_joint_2"<<"\t"<<"pos_joint_6"
                 <<"\t"<<"vel_joint_4"<<"\t"<<"vel_joint_5"<<"\t"<<"vel_joint_3"
                 <<"\t"<<"vel_joint_1"<<"\t"<<"vel_joint_2"<<"\t"<<"vel_joint_6"
                 <<"\t"<<"acc_joint_4"<<"\t"<<"acc_joint_5"<<"\t"<<"acc_joint_3"
                 <<"\t"<<"acc_joint_1"<<"\t"<<"acc_joint_2"<<"\t"<<"acc_joint_6"<<"\t"<<"time_from_start"<<endl;
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
        i++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_data_capture");
  ros::NodeHandle n;
  static trajectory_msgs::JointTrajectoryPoint actual_data;
  static int i;
  ofstream trajectoryFile;
  i=0;
  trajectoryFile.open("sensor_data.txt");
  ros::Subscriber sub = n.subscribe("RCB_full_mode_controller/state", 1000, chatterCallback);
  ros::spin();

  return 0;
}
