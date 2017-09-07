% type 6Mode_Redundant_0905_encoders_data.txt
% dlmread 
% textscan
% fgetl
% fscanf
% fileID = fopen('6Mode_Redundant_0905_encoders_data.txt','r');
% formatSpec = '%f';
% A = fscanf(fileID,formatSpec);
% fclose(fileID);

Time = ModeRedundant0905encodersdata.time_stamp(2:2975) - ModeRedundant0905encodersdata.time_stamp(2)*ones(2974,1); 

%% Actual sensor feedback
% Position
actual_pos_joint_4 = ModeRedundant0905encodersdata.actual_pos_joint_4(2:2975);
actual_pos_joint_5 = ModeRedundant0905encodersdata.actual_pos_joint_5(2:2975);
actual_pos_joint_3 = ModeRedundant0905encodersdata.actual_pos_joint_3(2:2975);
actual_pos_joint_1 = ModeRedundant0905encodersdata.actual_pos_joint_1(2:2975);
actual_pos_joint_2 = ModeRedundant0905encodersdata.actual_pos_joint_2(2:2975);
actual_pos_joint_6 = ModeRedundant0905encodersdata.actual_pos_joint_6(2:2975);

% Velocity
actual_vel_joint_4 = ModeRedundant0905encodersdata.actual_vel_joint_4(2:2975);
actual_vel_joint_5 = ModeRedundant0905encodersdata.actual_vel_joint_5(2:2975);
actual_vel_joint_3 = ModeRedundant0905encodersdata.actual_vel_joint_3(2:2975);
actual_vel_joint_1 = ModeRedundant0905encodersdata.actual_vel_joint_1(2:2975);
actual_vel_joint_2 = ModeRedundant0905encodersdata.actual_vel_joint_2(2:2975);
actual_vel_joint_6 = ModeRedundant0905encodersdata.actual_vel_joint_6(2:2975);

% Acceleration
actual_acc_joint_4 = ModeRedundant0905encodersdata.actual_acc_joint_4(2:2975);
actual_acc_joint_5 = ModeRedundant0905encodersdata.actual_acc_joint_5(2:2975);
actual_acc_joint_3 = ModeRedundant0905encodersdata.actual_acc_joint_3(2:2975);
actual_acc_joint_1 = ModeRedundant0905encodersdata.actual_acc_joint_1(2:2975);
actual_acc_joint_2 = ModeRedundant0905encodersdata.actual_acc_joint_2(2:2975);
actual_acc_joint_6 = ModeRedundant0905encodersdata.actual_acc_joint_6(2:2975);


%% Desired trajectory
% Position
desired_pos_joint_4 = ModeRedundant0905encodersdata.desired_pos_joint_4(2:2975);
desired_pos_joint_5 = ModeRedundant0905encodersdata.desired_pos_joint_5(2:2975);
desired_pos_joint_3 = ModeRedundant0905encodersdata.desired_pos_joint_3(2:2975);
desired_pos_joint_1 = ModeRedundant0905encodersdata.desired_pos_joint_1(2:2975);
desired_pos_joint_2 = ModeRedundant0905encodersdata.desired_pos_joint_2(2:2975);
desired_pos_joint_6 = ModeRedundant0905encodersdata.desired_pos_joint_6(2:2975);

% Velocity
desired_vel_joint_4 = ModeRedundant0905encodersdata.desired_vel_joint_4(2:2975);
desired_vel_joint_5 = ModeRedundant0905encodersdata.desired_vel_joint_5(2:2975);
desired_vel_joint_3 = ModeRedundant0905encodersdata.desired_vel_joint_3(2:2975);
desired_vel_joint_1 = ModeRedundant0905encodersdata.desired_vel_joint_1(2:2975);
desired_vel_joint_2 = ModeRedundant0905encodersdata.desired_vel_joint_2(2:2975);
desired_vel_joint_6 = ModeRedundant0905encodersdata.desired_vel_joint_6(2:2975);

% Acceleration
desired_acc_joint_4 = ModeRedundant0905encodersdata.desired_acc_joint_4(2:2975);
desired_acc_joint_5 = ModeRedundant0905encodersdata.desired_acc_joint_5(2:2975);
desired_acc_joint_3 = ModeRedundant0905encodersdata.desired_acc_joint_3(2:2975);
desired_acc_joint_1 = ModeRedundant0905encodersdata.desired_acc_joint_1(2:2975);
desired_acc_joint_2 = ModeRedundant0905encodersdata.desired_acc_joint_2(2:2975);
desired_acc_joint_6 = ModeRedundant0905encodersdata.desired_acc_joint_6(2:2975);


%% 
% Position
figure(1)
plot(Time(:,1),actual_pos_joint_4(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_4(:,1),'k-'); hold on;

plot(Time(:,1),actual_pos_joint_5(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_5(:,1),'k-'); hold on;

plot(Time(:,1),actual_pos_joint_3(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_3(:,1),'k-'); hold on;

plot(Time(:,1),actual_pos_joint_1(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_1(:,1),'k-'); hold on;

plot(Time(:,1),actual_pos_joint_2(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_2(:,1),'k-'); hold on;

plot(Time(:,1),actual_pos_joint_6(:,1)); hold on;
plot(Time(:,1),desired_pos_joint_6(:,1),'k-'); hold on;

%% Velocity
figure(2)
%plot(Time(:,1),actual_vel_joint_4(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_4(:,1),'k-'); hold on;

%plot(Time(:,1),actual_vel_joint_5(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_5(:,1),'k-'); hold on;

%plot(Time(:,1),actual_vel_joint_3(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_3(:,1),'k-'); hold on;

%plot(Time(:,1),actual_vel_joint_1(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_1(:,1),'k-'); hold on;

%plot(Time(:,1),actual_vel_joint_2(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_2(:,1),'k-'); hold on;

plot(Time(:,1),actual_vel_joint_6(:,1)); hold on;
plot(Time(:,1),desired_vel_joint_6(:,1),'k-'); hold on;
