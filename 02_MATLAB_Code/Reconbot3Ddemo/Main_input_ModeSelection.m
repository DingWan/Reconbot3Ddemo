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
 
% Intepotation Points and Time
NumIntepoPoints = 20;
Time = 4;

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
   
   % -------- if provious mode is 5 and current mode is 10 or 11 ----------
   if Mode_previous == 5 && Mode_current == 10
       % Special Treatment for Modes 10 and 11 (2R_Serial_AiCi)
       % Transition uses Mode 6 (Sixbar)
       Mode_current = 6;
       PosOri_current = {0, -29.999999999999996, 77.459666924148340, [], [], 0.739017879013918};
       %PosOri_transit = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{4};
       %PosOri_current = {PosOri_transit(1), PosOri_transit(2), PosOri_transit(3), [], [], PosOri_transit(6)};
   elseif Mode_previous == 5 && Mode_current == 11
       % Special Treatment for Modes 10 and 11 (2R_Serial_AiCi)
       % Transition uses Mode 6 (Sixbar)
       Mode_current = 6;
       PosOri_current = {0, 30.0000000000000, 77.4596669241483, [], [], 0.739017879013918};
   end
   % ----------------------------------------------------------------------
     
   % Motion Planning and Optimal Soultion;
   q0q1q2_P2P = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                              Mode_current, PosOri_current, q0q1q2_current, ...
                                              NumIntepoPoints,Time, l1, l2);
   q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P];
   
   % ----------------------------------------------------------------------
   % ---------- Adjust the q12/q22 from mode 5 to mode 10/11---------------
   if Mode_previous == 5 && Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{1} == 10
       Mode_previous = 10;
       Mode_current = 10;
       PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{2};
       
               % Keep the first angles  q12 and q14  equals to the second angles  q12 and q14
               %--- Here we assign the value by extrapolation ----
               % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
               % Here we get the new "q0q1q2_mat(Lenth,7:11)" value
               Lenth = length(q0q1q2_mat(:,1));
               delta_q0q1q2 = ((q0q1q2_mat(Lenth-2,:)-q0q1q2_mat(Lenth-1,:)) + (q0q1q2_mat(Lenth-3,:)-q0q1q2_mat(Lenth-2,:)))/2;
               q0q1q2_mat(Lenth,3) = q0q1q2_mat(Lenth-1,3) - delta_q0q1q2(3);
               PosOri_previous{7} = q0q1q2_mat(Lenth,3);
               po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
               obj2RserialA1C1 = RCB2RserialA1C1(po,[],l1,l2);
               [~, ~, ~, q1q2, ~] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
               for jj = 1:length(q1q2(:,1))
                   q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_mat(Lenth-1,2:6));
                   q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_mat(Lenth-1,7:11));
               end
               [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
               [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
               q0q1q2_mat(Lenth,1:11) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
       
       PosOri_previous{7} = q0q1q2_mat(length(q0q1q2_mat(:,1)),3);
       PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{2};
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
       q0q1q2_q12_SelfAdjustment = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                 Mode_current, PosOri_current, q0q1q2_current,  NumIntepoPoints,Time, l1, l2);
       q0q1q2_mat = [q0q1q2_mat; q0q1q2_q12_SelfAdjustment];
   elseif Mode_previous == 5 && Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{1} == 11
       Mode_previous = 11;
       Mode_current = 11;
       PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{2};
       
               % Keep the first angles  q12 and q14  equals to the second angles  q12 and q14
               %--- Here we assign the value by extrapolation ----
               % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
               % Here we get the new "q0q1q2_mat(Lenth,7:11)" value
               Lenth = length(q0q1q2_mat(:,1));
               delta_q0q1q2 = ((q0q1q2_mat(Lenth-2,:)-q0q1q2_mat(Lenth-1,:)) + (q0q1q2_mat(Lenth-3,:)-q0q1q2_mat(Lenth-2,:)))/2;
               q0q1q2_mat(Lenth,8) = q0q1q2_mat(Lenth-1,8) - delta_q0q1q2(8);
               PosOri_previous{7} = q0q1q2_mat(Lenth,8);
               po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
               obj2RserialA2C2 = RCB2RserialA2C2(po,[],l1,l2);
               [~, ~, ~, q1q2, ~] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
               for jj = 1:length(q1q2(:,1))
                   q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_mat(Lenth-1,2:6));
                   q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_mat(Lenth-1,7:11));
               end
               [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
               [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
               q0q1q2_mat(Lenth,1:11) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
       
       PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1}{2};
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
       q0q1q2_q22_SelfAdjustment = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                 Mode_current, PosOri_current, q0q1q2_current,  NumIntepoPoints,Time, l1, l2);
       q0q1q2_mat = [q0q1q2_mat; q0q1q2_q22_SelfAdjustment];
   end
   % -------------------------------------------------------------------
end
toc


IntepPointNum = IntepPointNum + 2;
%% Last step for returnning to HomePosition
tic
%---------------
for OnlyforFoldThisPart = 1:1
Mode_Pos_Ori_TrajPoints_cell{IntepPointNum} = { 5, {0 0 255.0445 0 [] [], 0 ,0},q0q1q2_HomePosition};
% First step: Calculate the next step and get the second row values of q11 and q21 after
% interpotation, and assign to the previous step.0
% Assgin Input value
Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1};
Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1};

PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{2};

q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat(:,1)),:);
q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{3};

% -------- if provious mode is 5 and current mode is 10 or 11 ----------
if Mode_current == 5 && (Mode_previous == 10 || Mode_previous == 11)
    % Special Treatment for Modes 10 and 11 (2R_Serial_AiCi)
    % Transition uses Mode 6 (Sixbar)
    Mode_previous = 6;
    PosOri_transit = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{4};
    PosOri_previous = {PosOri_transit(1), PosOri_transit(2), PosOri_transit(3), [], [], PosOri_transit(6)};
end
% ----------------------------------------------------------------------

% Motion Planning and Optimal Soultion;
   q0q1q2_P2P_HomePosition = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                           Mode_current, PosOri_current, q0q1q2_current, NumIntepoPoints,Time, l1, l2);
                                                       

   % ---------- Adjust the q12/q22 from mode 10/11 to mode 5 ---------------
   if  Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1} == 5 && Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1} == 10
       Mode_previous = 10;
       Mode_current = 10;
       PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
       PosOri_previous{7} = q0q1q2_mat(length(q0q1q2_mat(:,1)),3);
       PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
       PosOri_current{7} = q0q1q2_P2P_HomePosition(2,3);
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
       % Keep the first angles  q12 and q14  equals to the second angles  q12 and q14 
       %--- Here we assign the value by extrapolation ----
       % Delta = ((Third value - Second Value) + (Forth value - Third Value))/2 
       delta_q0q1q2 = ((q0q1q2_P2P_HomePosition(3,:)-q0q1q2_P2P_HomePosition(2,:)) + (q0q1q2_P2P_HomePosition(4,:)-q0q1q2_P2P_HomePosition(3,:)))/2;
       q0q1q2_P2P_HomePosition(1,3) = q0q1q2_P2P_HomePosition(2,3) - delta_q0q1q2(3);
       q0q1q2_P2P_HomePosition(1,5) = q0q1q2_P2P_HomePosition(2,5) - delta_q0q1q2(5);
       q0q1q2_q12_SelfAdjustment = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                 Mode_current, PosOri_current, q0q1q2_current,  NumIntepoPoints,Time, l1, l2);
       q0q1q2_mat = [q0q1q2_mat; q0q1q2_q12_SelfAdjustment; q0q1q2_P2P_HomePosition];
   elseif Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1} == 5 && Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1} == 11
       Mode_previous = 11;
       Mode_current = 11;
       PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
       PosOri_previous{7} = q0q1q2_mat(length(q0q1q2_mat(:,1)),8);
       PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
       PosOri_current{7} = q0q1q2_P2P_HomePosition(2,8);
       q0q1q2_previous = q0q1q2_mat(length(q0q1q2_mat),:);
       % Keep the first angles  q12 and q14  equals to the second angles  q12 and q14 
       %--- Here we assign the value by extrapolation ----
       % Delta = ((Third value - Second Value) + (Forth value - Third Value))/2 
       delta_q0q1q2 = ((q0q1q2_P2P_HomePosition(3,:)-q0q1q2_P2P_HomePosition(2,:)) + (q0q1q2_P2P_HomePosition(4,:)-q0q1q2_P2P_HomePosition(3,:)))/2;
       q0q1q2_P2P_HomePosition(1,8) = q0q1q2_P2P_HomePosition(2,8) - delta_q0q1q2(8);
       q0q1q2_P2P_HomePosition(1,10) = q0q1q2_P2P_HomePosition(2,10) - delta_q0q1q2(10);
       
       q0q1q2_q22_SelfAdjustment = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                 Mode_current, PosOri_current, q0q1q2_current,  NumIntepoPoints,Time, l1, l2);
       q0q1q2_mat = [q0q1q2_mat; q0q1q2_q22_SelfAdjustment; q0q1q2_P2P_HomePosition];
   else
       q0q1q2_mat = [q0q1q2_mat; q0q1q2_P2P_HomePosition];
   end
   % -------------------------------------------------------------------
end
% --------------
toc


%% Save the value as '.mat' file
% q0q1q2_mat_Angle = q0q1q2_mat * 180 / pi;
% save('q0q1q2_mat_Angle')
% q11q12q13q21q22q23_matrix_Angle = [q0q1q2_mat_Angle(:,2:4),q0q1q2_mat_Angle(:,7:9)];
% save('q11q12q13q21q22q23_matrix_Angle')

%% Plot joint Angles
%PlotAngleValue;

%% 3D Animation
for i = 21:length(q0q1q2_mat)-0    
    %========================== Animation ============================
    ReconbotANI(q0q1q2_mat(i,:));   
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

%clr_trail_CollisionPoints_button_press;
