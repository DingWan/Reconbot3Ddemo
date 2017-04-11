%%--------------------2-RER Mode selection-----------------------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 15 Nov 2016.
%     The copyright is belong to Wan Ding.

%       po = {cell}               Position and rotation of the MP, cell
%       po_cell = {cell}          Position and rotation of the MP, cell
%       po_num = {num}            Position and rotation of the MP, number
%       SolutionRow               The row of solution in 8 solutions.
%       EulerAngle_q11_theta      Euler Angle(Z-Y-X)  q11, and theta: [alpha, beta, gamma, q11, theta]

%%-------------------------------------------------------------------
clc
%close all
clear 
%clf

l1 = 230;
l2 = 147.25;
deg = pi/180;
n = 25;
Tension=0;
laststep = 0;

addpath(genpath(pwd)); % Enalbe all folders inside "SSoop"
%InitHome;       

%% Select Mode + Posture
SelectNumberOfTrajectoryPoints;
% load('InsertRowTransiConfig_Test.mat')

%% Motion planning
tic
q0q1q2_HomePosition = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0];
p_0 = [0 0 255.0445 0 0 0, 0 0];
q0q1q2_mat = [];
for IntepPointNum = 1 : NumTP
    
    if IntepPointNum == 1
        Mode = 5;
        Posture = [0 0 255.0445 0 0 0, 0 0];
    else        
    end
    
   % Assgin Input value
   Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum,1}{1};
   Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{1};
   
   PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum,1}{2};
   PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{2};
   
   if IntepPointNum == 1
       q0q1q2_previous = q0q1q2_HomePosition;
   else
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
   end
   q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{3};
   
   % Intepotation Points and Time
   NumIntepoPoints = 20;
   Time = 4;
   
   % Motion Planning and Optimal Soultion;
   q0q1q2_P2P = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                              Mode_current, PosOri_current, q0q1q2_current,  NumIntepoPoints,Time, l1, l2);
   q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P];
end
toc


IntepPointNum = IntepPointNum + 2;
%% Last step for returnning to HomePosition
tic
%---------------
Mode_Pos_Ori_TrajPoints_cell{IntepPointNum} = { 5, {0 0 255.0445 0 [] [], 0 ,0},q0q1q2_HomePosition};
% First step: Calculate the next step and get the second row values of q11 and q21 after
% interpotation, and assign to the previous step.0
% Assgin Input value
Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1,1}{1};
Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum,1}{1};

PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1,1}{2};
PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum,1}{2};

q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat(:,1)),:);
q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum,1}{3};

% Motion Planning and Optimal Soultion;
   q0q1q2_P2P_HomePosition = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                           Mode_current, PosOri_current, q0q1q2_current, NumIntepoPoints,Time, l1, l2);
q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P_HomePosition];
%---------------
toc

%% Save the value as '.mat' file
% q0q1q2_mat_Angle = q0q1q2_mat * 180 / pi;
% save('q0q1q2_mat_Angle')
% q11q12q13q21q22q23_matrix_Angle = [q0q1q2_mat_Angle(:,2:4),q0q1q2_mat_Angle(:,7:9)];
% save('q11q12q13q21q22q23_matrix_Angle')

%% Plot joint Angles
%PlotAngleValue;

%% 3D Animation
for i = 55:length(q0q1q2_mat)-18
    %========================== Animation ============================
    ReconbotANI(q0q1q2_mat(i,:));
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

%clr_trail_CollisionPoints_button_press;
