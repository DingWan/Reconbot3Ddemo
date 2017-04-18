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

q0q1q2_HomePosition = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0];
p_0 = [0 0 255.0445 0 0 0, 0 0];
q0q1q2_mat = [];
% Intepotation Points and Time
NumIntepoPoints = 20;
Time = 4;

addpath(genpath(pwd)); % Enalbe all folders inside "SSoop"
%InitHome;       

%% Select Mode + Posture
% SelectNumberOfTrajectoryPoints;

%------ Single Mode ----------
% load('q0q1q2_3T2R.mat')
% load('q0q1q2_3T1R.mat')
% load('q0q1q2_3T1R-SingularityA1C1.mat')
% load('q0q1q2_3T1R-SingularityA2C2.mat')
% load('q0q1q2_3T1R-SingularityA1C1A2C2.mat')
% load('q0q1q2_2T2R.mat')
% load('q0q1q2_1T3RRotAroundPoint.mat')
% load('q0q1q2_2T2Rfivebar.mat')
% load('q0q1q2_2T2Rthreebar.mat')
% load('q0q1q2_2RserialA1C1.mat')
% load('q0q1q2_2RserialA2C2.mat')

load('Modes_3to4.mat')

%% Motion planning
clc
q0q1q2_mat = [];
tic
for IntepPointNum = 1 : NumTP
    
    if IntepPointNum == 1
        Mode = 5;
        Posture = [0 0 255.0445 0 0 0, 0 0];
    else        
    end
    
   % Assgin Input value
   % Mode
   Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1};
   Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{1};
   % Position+Oritation
   PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{2};
   PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{2};
   % value of all angles
   if IntepPointNum == 1
       q0q1q2_previous = q0q1q2_HomePosition;
   else
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
   end
   q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{3};
        
   % Motion Planning and Optimal Soultion;
   q0q1q2_P2P = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                              Mode_current, PosOri_current, q0q1q2_current, ...
                                              NumIntepoPoints,Time, l1, l2);
   q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P];   
   
   if Mode_previous == 5 && Mode_current == 10
       q0q1q2_2RserialA1C1_x_0_y_30 = q0q1q2_mat(length(q0q1q2_mat)-NumIntepoPoints,:);
   elseif Mode_previous == 5 && Mode_current == 11
       q0q1q2_2RserialA2C2_x_0_y_30 = q0q1q2_mat(length(q0q1q2_mat)-NumIntepoPoints,:);
   end
end
toc

IntepPointNum = IntepPointNum + 2;
%% Last step for returnning to HomePosition
tic
%---------------
for OnlyUsedforFoldingThisPart = 1:1
    Mode_Pos_Ori_TrajPoints_cell{IntepPointNum} = { 5, {0 0 255.0445 0 [] [], 0 ,0},q0q1q2_HomePosition};
    % First step: Calculate the next step and get the second row values of q11 and q21 after
    % interpotation, and assign to the previous step.0
    % Assgin Input value
    Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1};
    Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1};
    
    PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
    PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{2};
    
    if Mode_previous == 10 && Mode_current  == 5
        q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 -3.14159265358979 0.112270797444876 3.14159265358979 0.944049245225854 -3.14159265358979 0 0.968872484986974 1.94286556262976 0.601923841807922 0];
        q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_mat(length(q0q1q2_mat(:,1)),:)];
    elseif Mode_previous == 11 && Mode_current  == 5
        q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0 0 0.968872484986974 1.94286556262976 0.601923841807922 0 -3.14159265358979 0.112270797444876 3.14159265358979 0.944049245225854 -3.14159265358979];
        q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_mat(length(q0q1q2_mat(:,1)),:)];    
    else
        q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat(:,1)),:);
    end
    q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{3};
    
    % Motion Planning and Optimal Soultion;
    q0q1q2_P2P_HomePosition = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
        Mode_current, PosOri_current, q0q1q2_current, NumIntepoPoints, Time, l1, l2);
    
    q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P_HomePosition];
end
% --------------
toc

%% Save the value as '.mat' file
% q0q1q2_mat_Angle = q0q1q2_mat * 180 / pi;
% save('q0q1q2_mat_Angle')

%% Plot joint Angles
%PlotAngleValue;

%% 3D Animation
for i = 41:length(q0q1q2_mat)-20   
    %========================== Animation ============================
    ReconbotANI(q0q1q2_mat(i,:));   
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

%clr_trail_CollisionPoints_button_press;
