function y=traj_pub(trajectory)
publisher = rospublisher('/reconbot_trajectory','control_msgs::FollowJointTrajectoryGoal');
pause(2);
send(publisher,trajectory);
pause(2);
end

