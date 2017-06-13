init_pos=data(1,:);
<<<<<<< HEAD
step1=data(1:100,:);
=======
step1=data(1:500,:);
>>>>>>> ParallelMechanismShape
step2=data(501:1000,:);
step3=data(1001:1500,:);
%%
final=data(500,:);
%%
robot.topic = '/reconbot_trajectory';
robot.jointNames = {'joint_4','joint_5','joint_3','joint_1','joint_2','joint_6'};
<<<<<<< HEAD
robot.pos = init_pos;
robot.timeFromStart = 5;
robot.trajPoints = robot.transform();
robot.tj = robot.buildTrajectory();
%%
robot.topic = '/reconbot_trajectory';
robot.jointNames = {'joint_4','joint_5','joint_3','joint_1','joint_2','joint_6'};
robot.pos = data;
robot.timeFromStart = 0.05;
robot.trajPoints = robot.transform();
%robot.tj = robot.buildTrajectory();
%%
robot.topic = '/reconbot_trajectory';
robot.jointNames = {'joint_4','joint_5','joint_3','joint_1','joint_2','joint_6'};
=======
robot.pos = data;
robot.timeFromStart = 0.05;
robot.trajPoints = robot.transform();
robot.tj = robot.buildTrajectory();
%%
>>>>>>> ParallelMechanismShape
robot.pos = step1;
robot.timeFromStart = 0.05;
robot.trajPoints = robot.transform();
robot.tj = robot.buildTrajectory();
%%
robot.pos = final;
robot.timeFromStart = 5;
robot.trajPoints = robot.transform();
robot.tj = robot.buildTrajectory();