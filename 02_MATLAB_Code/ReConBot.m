%% ReConBot Class for Matlab.
% This class allows to create objects which have the property to stablish
% comunication with the ReConBot Control package that runs in background in
% a Linux based machine.
% Author: Jorge De La Cruz and Wang Ding
% date: 13 March 2017
% version: 0.1
%% Definition Class
classdef ReConBot
    
    %% Object's Properties Definition
    % Here the proporties available for the object are defined.
    % _topic_: it is the name of the topic where the message is goint to be published. 
    % _jointNames_: name of the joints to plan for.
    % _tj_: ROS Trajectory message of the type
    % control_msg/FollowJointTratjectoryGol. This message is built using
    % the method buildTrajectory.
    
    properties
        topic;
        jointNames={};
        points=[];
        tj;
    end
    
    %% Methods of the Class ReConBot
    %Methods aimed to prepare the trajectory messages in Matlab and send it
    %to the ROS network in order to get connection with the ReConBot's
    %controller.
   
    methods
        function obj=ReConBot()
        end
        function traj=buildTrajectory(obj)
            traj = rosmessage('control_msgs/FollowJointTrajectoryGoal');
            traj.Trajectory.JointNames = obj.jointNames;
            [m,n]=size(obj.points);
            for i=1:m
                positions = obj.points(i,[1:3:n-1]);
                velocities = obj.points(i,[2:3:n]);
                accelerations = obj.points(i,[3:3:n]);
                timefromstart = rosduration(obj.points(i,n));  
                points_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                points_msg.Positions = positions;
                points_msg.Velocities = velocities;
                points_msg.Accelerations = accelerations;
                points_msg.TimeFromStart = timefromstart;
                %joint_trajectory(i) = points_msg;
                %traj.Trajectory.Points = joint_trajectory;              
                traj.Trajectory.Points(i) = points_msg;
                clear points_msg;
            end
        end
            
        function trajPub(obj)
            publisher = rospublisher(obj.topic, 'control_msgs/FollowJointTrajectoryGoal');
            pause(2);
            send(publisher,obj.tj);
            pause(2);
        end
        
        function subscriber=getTraj(obj)
            subscriber = rossubscriber('/reconbot_trajectory',@reconbotCallback);
        end
    end
end
            
            