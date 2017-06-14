clc
clear
l1 = 0.2301390;
l2 = 0.1477;

q0q1q2_HomePosition = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
p_0 = [0 0 0.208879343162506 0 0 0, 0 0]; %  p = 2 * l2 * sin(pi/4)

deg = pi/180;
addpath(genpath(pwd)); % Enalbe all folders inside "Reconbot3Ddemo"

%% Initialization
%rosinit
Reconbot = ReConBot;
Reconbot.topic = '/RCB_full_mode_controller/state';

%% Read the value of Motor Encoder

% tic
% DyInfo = Reconbot.getTraj();
% pause(0.025)
% DyEnPos = DyInfo.LatestMessage.Actual.Positions;
% MotorPosition = DyEnPos';
% toc

MotorPosition = [90*pi/180, 0*pi/180, -90*pi/180, 90*pi/180, 0*pi/180, 180*pi/180];

%% Intepotation Points and Time
NumIntepoPoints = 20;
Start_Time = 0;
Time_inteval = 5;
%
q0q1q2_Pos_mat = [];
q0q1q2_Vel_mat = [];
q0q1q2_Acc_mat = [];
%
MP_Pos_mat = [];
MP_Vel_mat = [];
MP_Acc_mat = [];
% 
MP_Pos_Intep = [];
MP_Vel_Intep = [];
MP_Acc_Intep = [];
MP_time_Intep = [];
%
q0q1q2_P2P_Pos_Intep = [];
q0q1q2_P2P_Vel_Intep = [];
q0q1q2_P2P_Acc_Intep = [];
q0q1q2_P2P_time_Intep = [];
%
Time_mat = [];
%
Mode_det_Jq_J_mat = [];
Mode_det_Jq_J = [];
Mode_det_Jq_J_HomePosition_mat = [];
Mode_det_Jq_J_HomePosition = [];
%
PosOri_Output_mat = [];

%% Check the Current Mode
q0q1q2_FixedMode = [0, 0, 0, pi, -pi/2, 0, 0, 0, -pi/2, 0, 0];
RandomMode = { 12, {0 0 0 [] [] [], MotorPosition(1), MotorPosition(2), MotorPosition(4), MotorPosition(5)}, q0q1q2_FixedMode};
%
po = RandomMode{2};
q11q12q21q22 = [];
objRCBFixedSerialChain = RCBFixedSerialChain(po,q11q12q21q22,l1,l2);
[p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = objRCBFixedSerialChain.RCB_FixedSerialChain_IK;
ReconbotANI([0 q1q2]); 

%% Random Configuration for returnning to HomePosition
tic
%---------------
for OnlyUsedforFoldingThisPart = 1:1
    
    Mode_Pos_Ori_TrajPoints_cell{1} = RandomMode;
    Mode_Pos_Ori_TrajPoints_cell{2} = { 5, {0 0 0.208879343162506 0 [] [], 0 ,0},q0q1q2_HomePosition};
    % First step: Calculate the next step and get the second row values of q11 and q21 after
    % interpotation, and assign to the previous step.0
    % Assgin Input value
    Mode_previous = Mode_Pos_Ori_TrajPoints_cell{1}{1};
    Mode_current  = Mode_Pos_Ori_TrajPoints_cell{2}{1};
    
    PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{1}{2};
    PosOri_current = Mode_Pos_Ori_TrajPoints_cell{2}{2};
    
    if Mode_previous == 10 && Mode_current  == 5
        q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0,-3.14159265358979,0.112488839324410,3.14159265358979,-0.944501238674810,-3.14159265358979,0,0.966880969134940,1.94661464276441,-0.603915357659957,0];
        q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
    elseif Mode_previous == 11 && Mode_current  == 5
        q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0,0,0.966880969134940,1.94661464276441,-0.603915357659957,0,-3.14159265358979,0.112488839324410,3.14159265358979,-0.944501238674810,-3.14159265358979];
        q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];    
    else
        q0q1q2_previous = [0, q1q2];
    end
    q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{2}{3};
    
    % Motion Planning and Optimal Soultion;
        [ PosOri_Output, q0q1q2_P2P_Pos_Intep_HomePosition, q0q1q2_P2P_Vel_Intep_HomePosition ,q0q1q2_P2P_Acc_Intep_HomePosition, ...
          MP_Pos_Intep_HomePosition, MP_Vel_Intep_HomePosition, MP_Acc_Intep_HomePosition, MP_time_Intep_HomePosition, Mode_det_Jq_J_HomePosition ] = ...
                                          MotionPlanningTransitionModes(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                                  Mode_current, PosOri_current, q0q1q2_current, ...
                                                                                  NumIntepoPoints, Start_Time, Time_inteval, l1, l2);  
                                                                              
    % Moving Platform Position
       PosOri_Output_mat = [PosOri_Output_mat; PosOri_Output];
            
    % Position, Velocity, Acceleration Calcuation of joint angles                                                                                                       NumIntepoPoints, Start_Time, Time_inteval, l1, l2);                                      
      q0q1q2_Pos_mat = [ q0q1q2_Pos_mat; q0q1q2_P2P_Pos_Intep_HomePosition ];
      q0q1q2_Vel_mat = [ q0q1q2_Vel_mat; q0q1q2_P2P_Vel_Intep_HomePosition ];
      q0q1q2_Acc_mat = [ q0q1q2_Acc_mat; q0q1q2_P2P_Acc_Intep_HomePosition ];
    % Position, Velocity, Acceleration Calcuation of Moving Platform   
          MP_Pos_mat = [ MP_Pos_mat; MP_Pos_Intep_HomePosition ];
          MP_Vel_mat = [ MP_Vel_mat; MP_Vel_Intep_HomePosition ];
          MP_Acc_mat = [ MP_Acc_mat; MP_Acc_Intep_HomePosition ];
            Time_mat = [ Time_mat; MP_time_Intep_HomePosition ];
     % Mode, Jacobian of Jq and J
   Mode_det_Jq_Jc_J_mat = [Mode_det_Jq_J_mat; Mode_det_Jq_J_HomePosition];
            
end
% --------------
toc

%% Save the value as '.mat' file
    Len_q0q1q2_mat = length(q0q1q2_Pos_mat);
    q11q12q14_q21q22q23 = [ q0q1q2_Pos_mat(:,2), q0q1q2_Vel_mat(:,2), q0q1q2_Acc_mat(:,2),...
                            q0q1q2_Pos_mat(:,3), q0q1q2_Vel_mat(:,3), q0q1q2_Acc_mat(:,3),...
                            q0q1q2_Pos_mat(:,5), q0q1q2_Vel_mat(:,5), q0q1q2_Acc_mat(:,5),...
                            q0q1q2_Pos_mat(:,7), q0q1q2_Vel_mat(:,7), q0q1q2_Acc_mat(:,7),...
                            q0q1q2_Pos_mat(:,8), q0q1q2_Vel_mat(:,8), q0q1q2_Acc_mat(:,8),...
                            q0q1q2_Pos_mat(:,9), q0q1q2_Vel_mat(:,9), q0q1q2_Acc_mat(:,9),...
                            Time_mat(:,1), Mode_det_Jq_Jc_J_mat
                          ];

%% 3D Animation
for i = 1:length(q0q1q2_Pos_mat)-0  
    %========================== Animation ============================
    ReconbotANI(q0q1q2_Pos_mat(i,:));   
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

 
