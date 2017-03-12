clear all
points = [2 4 5 6 7 5 4 3 2 4; 9 3 4 2 3 1 6 5 4 8];
traj = rosmessage('control_msgs/FollowJointTrajectoryGoal');
points_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
points_msg2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');

%storage = rosmessage('trajectory_msgs/JointTrajectory');
traj.Trajectory.JointNames = {'joint_1', 'joint_2', 'joint_3'};
[m,n]=size(points);
            for i=1:1:m
                positions = points(i,[1:3:n-1]);
                velocities = points(i,[2:3:n]);
                accelerations = points(i,[3:3:n]);
                timefromstart = rosduration(points(i,n));  
%                 positions2 = points(i+1,[1:3:n-1]);
%                 velocities2 = points(i+1,[2:3:n]);
%                 accelerations2 = points(i+1,[3:3:n]);
%                 timefromstart2 = rosduration(points(i+1,n));
                points_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                points_msg.Positions = positions;
                points_msg.Positions;
                points_msg.Velocities = velocities;
                points_msg.Accelerations = accelerations;
                points_msg.TimeFromStart = timefromstart;
                
%                 points_msg2.Positions = positions2;
%                 points_msg2.Velocities = velocities2;
%                 points_msg2.Accelerations = accelerations2;
%                 points_msg2.TimeFromStart = timefromstart2;
                traj.Trajectory.Points(i)=points_msg
                clear points_msg
                %storage.Points.Positions
                %a=[points_msg,points_msg2]
%                 a(1)=points_msg;
%                 a(2)=points_msg2

      
            end
            %storage.Positions
            %points_msg.Positions;
            %traj.Trajectory=storage;
