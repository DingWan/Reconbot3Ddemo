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
% load('InsertRowTransiConfig_Test.mat')0
%  load('TwoSingularityA2C2.mat')
%  load('Mode2T2Rsixbar.mat')

%% Transition Strategy
%[MPOTP_cell, Modes_q0q1q2_cell] = TransitionStrategy(TrajPointsAndOperationalModes, Modes_q0q1q2_cell, Mode_Pos_Ori_TrajPoints_cell, NumTP);  

%% Value of q0q1q2 = [q0, q11-q15, q21-q25], PosOri = [Px, Py, Pz, alpha, beta, gamma]
% tic
% for IntepPointNum = 1 : length(MPOTP_cell)
%         
%     Mode = MPOTP_cell{IntepPointNum}{1};
%     po = { MPOTP_cell{IntepPointNum}{2:length(MPOTP_cell{IntepPointNum})} }; % 3T1R mode - self-adjustment
%     q0 = 0;
%      
%     %%----------------------- Calculate RCB IK -------------------------
%     switch Mode
%         case 1 % 3T2R
%             q11q12q21q22 = [];
%             obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
%         case 2 % 3T1R
%             % 3T1R mode:  [1 1 1 1 0 0]
%             % p = [x, y, z, alpha, [], []];
%             q11q12q21q22 = [];
%             obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;            
%         case 3 % 3T1R-SingularityA1C1
%             q11q12q21q22 = [];
%             obj3T1R = RCB3T1RSingularityA1C1(po, q11q12q21q22, l1, l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
%         case 4 % 3T1R-SingularityA2C2
%             q11q12q21q22 = [];
%             obj3T1R = RCB3T1RSingularityA2C2(po, q11q12q21q22, l1, l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
%         case 5 % 3T1R-SingularityA1C1A2C2
%             q11q12q21q22 = [];
%             obj3T1R = RCB3T1RSingularityA1C1A2C2(po, q11q12q21q22, l1, l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1A2C2_IK;
%         case 6 % 2T2R-6-Bar
%             % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
%             % p = [x, y, z, [], [], gamma]
%             q11q12q14q23 = [];
%             obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
%         case 7 % 2T2R-6-Bar(xy=0)
%             % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
%             % p = [[], [], z, [], beta, gamma]; x = y = 0
%             q11q12q14q23 = [];
%             obj1T2RRotAroundPoint = RCB1T3RRotAroundPoint(po,q11q12q14q23,l1,l2);
%             [EulerAngle_q11_theta, ABC, q1q2] = obj1T2RRotAroundPoint.RCB_1T2R_RotAroundPoint_IK;
%         case 8 % 2T2R-5-Bar
%             % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
%             % p = [x, 0, z, [], beta, 0]
%             q11q12q14q22 = [];
%             obj2T2Rfivebar = RCB2T2Rfivebar(po,q11q12q14q22,l1,l2);
%             [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK;
%         case 9 % 2T1R-3-BarSerial
%             % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
%             % p = [x, 0, z, 0, beta, []]
%             [EulerAngle_q11_theta, ABC, q1q2] = RCB_ThreeBar_IK(po, l1, l2);
%         case 10 % 2R-SerialA1C1
%             % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
%             % p = [x, y, [], [], [], []]; y < 0
%             [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(po, l1, l2);
%         case 11 % 2R-SerialA2C2
%             % Four-bar linkage with Serial Chain A2C2: [1 1 0 0 0 0]
%             % p = [x, y, [], [], [], []]; y > 0
%             [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA2C2_IK(po, l1, l2);
%         case 12 % Fixed-SerialA1C1A2C2
%             % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
%             % p = [0, 0, 0, [], [], []]
%             [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
%     end
% 
%     %%--------------------------- Method: End -----------------------------
%     p_cell{IntepPointNum,:} = p;
%     EulerAngle_q11_theta_cell{IntepPointNum,:} = EulerAngle_q11_theta;
%     ABC_cell{IntepPointNum,:} = ABC;    
%     
%     q0q1q2_EachTrajPoint = [zeros(length(q1q2(:,1)),1), q1q2];
%     q0q1q2_cell{IntepPointNum,:} = q0q1q2_EachTrajPoint;
% end
% toc

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
   NumIntepoPoints = 100;
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
for i = 1:length(q0q1q2_mat)-0
    %========================== Animation ============================
    ReconbotANI(q0q1q2_mat(i,:));
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

%clr_trail_CollisionPoints_button_press;
