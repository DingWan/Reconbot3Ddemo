
%%
data = Origin_Redundant_q1q2; %(1:100,:)
% data = Replan_q1q2;

%%
RandomFixed2Home_Expts;
data = Fixed2Home_q1q2;

%%
% rosinit
robot = ReConBot;
robot.topic = '/reconbot_trajectory';
robot.jointNames = {'joint_4','joint_5','joint_3','joint_1','joint_2','joint_6'};
robot.pos = data;
robot.timeFromStart = 5;
robot.trajPoints = robot.transform();
%robot.tj = robot.buildTrajectory();


%%
% figure(1)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_4); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_4); hold on
% 
% figure(2)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_5); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_5); hold on
% 
% figure(3)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_3); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_3); hold on
% 
% figure(4)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_1); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_1); hold on
% 
% figure(5)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_2); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_2); hold on
% 
% figure(6)
% plot(sensordata.time_stamp,sensordata.actual_pos_joint_6); hold on
% plot(sensordata.time_stamp,sensordata.desired_pos_joint_6); hold on