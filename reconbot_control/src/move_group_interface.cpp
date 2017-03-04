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


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class ReConBotArm
{
private:
  TrajClient* traj_client_;
public:
  ReConBotArm()
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("reconbot_arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }
  //! Clean up the action client
  ~ReConBotArm()
  {
  delete traj_client_;
  }
  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
      // When to start the trajectory: 1s from now
      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      traj_client_->sendGoal(goal);
    }


  control_msgs::FollowJointTrajectoryGoal armTrajectory()
    {
      //our goal variable
      control_msgs::FollowJointTrajectoryGoal goal;

      // First, the joint names, which apply to all waypoints
      goal.trajectory.joint_names.push_back("joint_2");
      goal.trajectory.joint_names.push_back("joint_3");
      goal.trajectory.joint_names.push_back("joint_6");

      // We will have two waypoints in this goal trajectory
      goal.trajectory.points.resize(2);

      // First trajectory point
      // Positions
      int ind = 0;
      goal.trajectory.points[ind].positions.resize(3);
      goal.trajectory.points[ind].positions[0] = 0.0;
      goal.trajectory.points[ind].positions[1] = 0.0;
      goal.trajectory.points[ind].positions[2] = 0.0;
      // Velocities
      goal.trajectory.points[ind].velocities.resize(3);
      for (size_t j = 0; j < 3; ++j)
      {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      // To be reached 1 second after starting along the trajectory
      goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

      // Second trajectory point
      // Positions
      ind += 1;
      goal.trajectory.points[ind].positions.resize(3);
      goal.trajectory.points[ind].positions[0] = 0.5;
      goal.trajectory.points[ind].positions[1] = 0.5;
      goal.trajectory.points[ind].positions[2] = 0.5;
      // Velocities
      goal.trajectory.points[ind].velocities.resize(3);
      for (size_t j = 0; j < 3; ++j)
      {
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      // To be reached 2 seconds after starting along the trajectory
      goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

      //we are done; return the goal
      return goal;
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
      return traj_client_->getState();
    }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "ReConBot_Driver");

  ReConBotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
