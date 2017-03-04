#*********************************************************************
# * Software License Agreement (BSD License)
# *
# *  Copyright (c) 2015, IGM-RWTH Aachen University
# *  All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *   * Neither the name of SRI International nor the names of its
# *     contributors may be used to endorse or promote products derived
# *     from this software without specific prior written permission.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
# *********************************************************************/

#  \authors Jorge De La Cruz and Alap Kshirsagar
#  \version 1.1
#  \date February 22, 2017

#!/usr/bin/env python
import roslib
roslib.load_manifest('dynamixel_reconbot')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')


        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            char = self.name[0] #either 'f' or 'b'
            goal.trajectory.joint_names = ['joint_2', 'joint_3','joint_6']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(0.5)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)


def main():
            arm = Joint('arm_reconbot')
            arm.move_joint([0.0,0.0,0.0])
            arm.move_joint([0.5,0.5,0.5])
            arm.move_joint([1.0,1.0,1.0])
            arm.move_joint([1.5,1.5,1.5])
            arm.move_joint([2.0,2.0,2.0])


if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
