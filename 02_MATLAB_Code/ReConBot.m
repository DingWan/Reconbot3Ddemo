%% ReConBot Class for Matlab.
% This class allows to create objects or instances which have the property to stablish
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
    % _control_msg/FollowJointTratjectoryGol_. This message is built using
    % the method _buildTrajectory_.

    properties
        topic='';
        jointNames={};
        trajPoints=[];
        pos = [];
        timeFromStart ;
        tj;
    end

    %% Methods of the Class ReConBot
    % Methods aimed to prepare the trajectory messages in Matlab and send it
    % to the ROS network in order to get connection with the ReConBot's
    % controller.

    methods
        % Function used for creating a new object or instance belongs the class
        % ReConBot. So, on the prompt:
        %
        % >> objName = ReConBot
        %
        % The above commad creates a new instance of the class
        % _ReConBot_ called _objName_
        %
        function obj=ReConBot()
        end

        function trajec=transform(obj)
          [m,n] = size(obj.pos);
          trajec(:, 1) = obj.pos(:, 1);
          trajec(:, 4) = obj.pos(:, 4);
          %trajec(:, 7) = 1.8611*pi-obj.pos(:, 3);
          trajec(:, 7) = -obj.pos(:, 7);
          %trajec(:, 10) = 0.5*pi+obj.pos(:, 4);
          trajec(:, 10) = obj.pos(:, 10);
          trajec(:, 13) = obj.pos(:, 13);
          %trajec(:, 16) = 1.1389*pi-obj.pos(:, 6);
          trajec(:, 16) = -obj.pos(:, 16);
          i=1;
          for i=1:m
            trajec(i, 19) = i*obj.timeFromStart;
          end
        end


        %% The buildTrajectory Function
        % The _buildTrajectory_ method builds the _FollowJointTrajectoryGoal_
        % message which is the message it must be sent to the ReConBot
        % controller. The function returns a trajectory of the type
        % _FollowJointTrajectoryGoal_, this trajectory must be passed to
        % the object property _tj_. Before creating the trajectory, the
        % trajectory points vector must be specified. This vector hast to be
        % passed to the instance (_objName_) throught the propery _points_.
        % For instance, for two desired trajectory points:
        %
        % >> points = [ 3 4 2 3 7 9 0 9 9 4; 7 2 3 4 2 4 3 4 2 8];
        %
        % and then,
        %
        % >> objName.points = points
        %
        % The joint names have to be defined into the object property
        % _jointNames_:
        %
        % >> objName.jointNames = {'joint_1', 'joint_2', 'joint_3'}
        %
        % Finally, the trajectory message can be built:
        %
        % >> objName.tj = buildTrajectory(objName)
        %
        function traj=buildTrajectory(obj)
            traj = rosmessage('control_msgs/FollowJointTrajectoryGoal');
            traj.Trajectory.JointNames = obj.jointNames;
            [m,n]=size(obj.trajPoints);
            for i=1:m
                positions = obj.trajPoints(i,1:3:n-1);
                velocities = obj.trajPoints(i,2:3:n);
                accelerations = obj.trajPoints(i,3:3:n);
                timefromstart = rosduration(obj.trajPoints(i,n));
                points_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                points_msg.Positions = positions;
                points_msg.Velocities = velocities;
                points_msg.Accelerations = accelerations;
                points_msg.TimeFromStart = timefromstart;
                traj.Trajectory.Points(i) = points_msg;
                clear points_msg;
                if i==m
                   disp('FollowJointTrajectoyGoal message successfully built.')
                end
            end
        end

        %% The trajPub Function
        % This function is necessary for publishing the trajectory built
        % with the _buildTrajectory_ function in the topic given in _topic_. Typing on the prompt:
        %
        % >> objName.topic = '/reconbot_trajectory'
        % >> trajPub(objName)
        %
        % Once executed the above command, the desired trajectory will be
        % sent to the /reconbot_trajectory topic or anyother it has
        % being specified in _topic_.

        function trajPub(obj)
            publisher = rospublisher(obj.topic, 'control_msgs/FollowJointTrajectoryGoal');
            pause(2);
            send(publisher,obj.tj);
            fprintf('Trajectory Message published on %s topic \n', obj.topic)
        end

        %% The getTraj Function
        % Return a Matlab message with the FollowJointTrajectoryGoal message
        % within the variable 'LatestMessage'. Therefore, the following
        % commands have to be typed for getting access to this message:
        %
        % >> trajectoryMatlab = getTraj(ObjName)
        % >> trajectoryROS=trajectoryMatlab.LastestMessage

        function inTraj=getTraj(obj)
            inTraj = rossubscriber(obj.topic,@reconbotCallback);
        end
    end
end
