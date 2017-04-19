% 2-RER PM Inverse kinematics
% 

% this file deals with IK of the 2-RER PM
% 
%%--------------------2-RER inverse kinematics-----------------------
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
clear
clf

%%---------------- Inputs ----------------
l1 = 2.20;
l2 = 1.4725;
deg = pi/180;
n = 25;
Tension=0; 
%% ----------------------Create Symbolic Variables--------------------------
% A1-C1
A1C1(1) = Link([0      0.6     0       pi/2]); 
A1C1(2) = Link([0      0       l2       0]);
A1C1(3) = Link([0      0       l2       0]);
A1C1(4) = Link([0      0       0       pi/2]);
A1C1(5) = Link([0      0.6     0       0]);
A1C1(6) = Link([0      0       l1       pi/2]);
BranchA1C1 = SerialLink(A1C1,'name','BranchA1C1');
BranchA1C1.base = transl(0, -l1/2, -0.6);
% A2-C2
A2C2(1) = Link([0      0.6     0       pi/2]); 
A2C2(2) = Link([0      0       l2       0]);
A2C2(3) = Link([0      0       l2       0]);
A2C2(4) = Link([0      0       0       pi/2]);
A2C2(5) = Link([0      0.6     0       0]);
A2C2(6) = Link([0      0       l1       pi/2]);
BranchA2C2 = SerialLink(A1C1,'name','BranchA2C2');
BranchA2C2.base = transl(0, l1/2, -0.6);


% RCB_Inputs;               
load('SplineTotal.mat');
tic
%%
for IntepPointNum = 1 : NumTP-1
    %%-------------Step 1st: Calculate the SplinePoints: i and i+1 ----------------------
    for i = 1:2
        p_Spline(i,:) = [Spline_Points(IntepPointNum + i -1,2:19), Spline_Points(IntepPointNum + i - 1, 8:19)];%[Spline_Points(IntepPointNum + i -1,2:19), zeros(1,12)];
    end    
    
    %%-------------Step 2nd: choose spline times: 1 or 2(mode change) -------------------
    if Spline_Points(IntepPointNum,1) == Spline_Points(IntepPointNum+1,1)
        SplineTimes = 1;
    else
        SplineTimes = 2;    
    end    
    % i = 2
    for i = 1:SplineTimes
        if i == 1
            if Spline_Points(IntepPointNum,1) ~= Spline_Points(IntepPointNum+1,1)
                if IntepPointNum == 1 
                     % As there are only 2 spline points
                    if Spline_Points(IntepPointNum,1) == 3
                        p_Spline(3,:) = [0, 0, Spline_Points(IntepPointNum + 1, 4), 0, 0, 0, Spline_Points(IntepPointNum, 8:19), Spline_Points(IntepPointNum + 1, 8:19)];
                    elseif Spline_Points(IntepPointNum,1) == 4 || Spline_Points(IntepPointNum,1) == 5
                        p_Spline(3,:) = [0, 0, Spline_Points(IntepPointNum + 1, 4), 0, 0, 0, Spline_Points(IntepPointNum, 8:19), Spline_Points(IntepPointNum + 1, 8:19)];
                    elseif Spline_Points(IntepPointNum + 1,1) == 5
                        p_Spline(3,:) = [0, 0, Spline_Points(IntepPointNum + 1, 4), 0, 0, 0, Spline_Points(IntepPointNum, 8:19), Spline_Points(IntepPointNum + 1, 8:19)];
                    else                       
                        p_Spline(3,:) = [Spline_Points(IntepPointNum + 1, 2:4), 0, 0, 0, Spline_Points(IntepPointNum, 8:19), zeros(1,12)];
                    end
                else
                    % As there are many steps more than 2 spline points
                    if Spline_Points(IntepPointNum,1) == 3 || Spline_Points(IntepPointNum,1) == 4 || Spline_Points(IntepPointNum,1) == 5
                        p_Spline(3,:) = [0, 0, Spline_Points(IntepPointNum + 1 ,4), 0, 0, 0, q1q2A1C1A2C2_lastValues, Spline_Points(IntepPointNum + i - 1, 8:19)];
                    else
                        p_Spline(3,:) = [Spline_Points(IntepPointNum + 1 ,2:4), 0, 0, 0, q1q2A1C1A2C2_lastValues, zeros(1,12)];
                    end
                    p_Spline(1,:) = [p_Spline(i,1:6), q1q2A1C1A2C2_lastValues, zeros(1,12)];                    
                end
                p_Spline([2;3],:) = p_Spline([3;2],:);% exchange rows 2 and 3
            else
                % No need to something
            end
            
        else
            if  Spline_Points(IntepPointNum,1) == 3 || Spline_Points(IntepPointNum,1) == 4 || Spline_Points(IntepPointNum,1) == 5 || ...
                     Spline_Points(IntepPointNum + 1,1) == 3 || Spline_Points(IntepPointNum + 1,1) == 5% || Spline_Points(IntepPointNum + 1,1) == 5                
                p_Spline(i,:) = [0, 0, Spline_Points(IntepPointNum + i - 1, 4), 0, 0, 0, q1q2A1C1A2C2_lastValues, Spline_Points(IntepPointNum + i - 1, 8:19)];
            else
                p_Spline(i,:) = [Spline_Points(IntepPointNum + i - 1,2:4), 0, 0, 0, q1q2A1C1A2C2_lastValues, Spline_Points(IntepPointNum + i - 1, 8:19)];
            end
        end
        addpath(genpath(pwd));
        %%-------------Step 3rd: Excute spline times: 1 or 2(mode ch    ange) -------------------
        switch Spline_Points(IntepPointNum + i -1,1)
            case 1
                %%------------------------ 3T1R IK ------------------------
                % 3T1R mode:  [1 1 1 1 0 0]
                % p = [x, y, z, alpha, [], []];
                    [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_3T1R(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 2
                %%------------------------ 2T2R IK: Six-bar ------------------------
                % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]    
                % p = [x, y, z, [], [], gamma]
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_2T2R_SixBar_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 3
                %%------------------------ 2T2R IK��Rotate around point  ------------------------
                % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
                % p = [[], [], z, [], beta, gamma]; x = y = 0
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_1T2R_RotAroundPoint_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 4
                %%------------------------ 2T1R IK��Planar Five-bar ------------------------
                % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
                % p = [x, 0, z, [], beta, 0]
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_FiveBar_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 5
                %%------------------------- 2T1R IK��Planar Three-bar -----------------------
                % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
                % p = [x, 0, z, 0, beta, []]
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_ThreeBar_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 6
                %-------------------------- Branch Chain A1C1 --------------------------------
                % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                % p = {x, y, [], [], [], []}; y < 0
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_2R_SerialA1C1_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 7
                %-------------------------- Branch Chain A2C2 --------------------------------
                % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                % p = {x, y, [], [], [], []}; y > 0
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_2R_SerialA2C2_IK(p_Spline(i:i+1,:), n, Tension, l1, l2);
            case 8
                %%------------------------- Two Serial Chains -----------------------------
                % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
                % p = {0, 0, 0, [], [], []}
                [p_Spline_TrajPlan, q1q2_traj, q1q2A1C1A2C2_lastValues] = RCBIK_TrajPlan_3T1R(p_Spline(i:i+1,:), n, Tension, l1, l2);
        end
        p_Spline_Total{i,:} = p_Spline_TrajPlan;
        q1q2_Total_cell{(IntepPointNum - 1) * 2 + i,:} = q1q2_traj;
    end    
end
toc

q1q2_Total_Matrix = cell2mat(q1q2_Total_cell);
save('q1q2_Total_Matrix.mat','q1q2_Total_Matrix');

%%-------------Step 4th: Excute spline times: 1 or 2(mode change) -------------------





 