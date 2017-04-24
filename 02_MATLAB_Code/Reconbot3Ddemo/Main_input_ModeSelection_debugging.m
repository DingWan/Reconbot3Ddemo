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
%                        Version 1.1
%%-------------------------------------------------------------------

clc
%close all
clear 
%clf

l1 = 230.0692;
l2 = 146.25;
deg = pi/180;

q0q1q2_HomePosition = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0];
p_0 = [0 0 253.3124 0 0 0, 0 0];

% Intepotation Points and Time
% q0q1q2_mat = [];
% NumIntepoPoints = 20;
% Time = 4;0

addpath(genpath(pwd)); % Enalbe all folders inside "SSoop"
%InitHome;       

%% Select Mode + Posture
SelectNumberOfTrajectoryPoints;

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

%  load('Modes_3to4.mat')

%% Motion planning
clc
% Intepotation Points and Time
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

tic
for IntepPointNum = 1 : NumTP
    
    if IntepPointNum == 1
        Mode = 5;
        Posture = [0 0 253.3124 0 0 0, 0 0];
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
       q0q1q2_previous = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat),:);
   end
   q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{3};
        
   % Motion Planning and Optimal Soultion;
    [ q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, ...
      MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep ] = MotionPlanningOptimalSoultion_debugging(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                                                          Mode_current, PosOri_current, q0q1q2_current, ...
                                                                                                          NumIntepoPoints, Start_Time, Time_inteval, l1, l2);                                      
    % Position, Velocity, Acceleration Calcuation of joint angles  
      q0q1q2_Pos_mat = [ q0q1q2_Pos_mat; q0q1q2_P2P_Pos_Intep ];
      q0q1q2_Vel_mat = [ q0q1q2_Vel_mat; q0q1q2_P2P_Vel_Intep ];
      q0q1q2_Acc_mat = [ q0q1q2_Acc_mat; q0q1q2_P2P_Acc_Intep ];
    % Position, Velocity, Acceleration Calcuation of Moving Platform   
          MP_Pos_mat = [ MP_Pos_mat; MP_Pos_Intep ];
          MP_Vel_mat = [ MP_Vel_mat; MP_Vel_Intep ];
          MP_Acc_mat = [ MP_Acc_mat; MP_Acc_Intep ];
            Time_mat = [ Time_mat; MP_time_Intep ];
            
   Start_Time = Time_mat(length(Time_mat(:,1)),1);
   
   if Mode_previous == 5 && Mode_current == 10
       q0q1q2_2RserialA1C1_x_0_y_30 = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat)-NumIntepoPoints,:);
   elseif Mode_previous == 5 && Mode_current == 11
       q0q1q2_2RserialA2C2_x_0_y_30 = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat)-NumIntepoPoints,:);
   end
end
toc

IntepPointNum = IntepPointNum + 2;
%% Last step for returnning to HomePosition
tic
%---------------
for OnlyUsedforFoldingThisPart = 1:1
    Mode_Pos_Ori_TrajPoints_cell{IntepPointNum} = { 5, {0 0 253.3124 0 [] [], 0 ,0},q0q1q2_HomePosition};
    % First step: Calculate the next step and get the second row values of q11 and q21 after
    % interpotation, and assign to the previous step.0
    % Assgin Input value
    Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1};
    Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1};
    
    PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
    PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{2};
    
    if Mode_previous == 10 && Mode_current  == 5
        q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 -3.14159265358979 0.112270797444876 3.14159265358979 0.944049245225854 -3.14159265358979 0 0.968872484986974 1.94286556262976 0.601923841807922 0];
        q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
    elseif Mode_previous == 11 && Mode_current  == 5
        q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0 0 0.968872484986974 1.94286556262976 0.601923841807922 0 -3.14159265358979 0.112270797444876 3.14159265358979 0.944049245225854 -3.14159265358979];
        q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];    
    else
        q0q1q2_previous = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:);
    end
    q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{3};
    
    % Motion Planning and Optimal Soultion;
        [ q0q1q2_P2P_Pos_Intep_HomePosition, q0q1q2_P2P_Vel_Intep_HomePosition ,q0q1q2_P2P_Acc_Intep_HomePosition, ...
          MP_Pos_Intep_HomePosition, MP_Vel_Intep_HomePosition, MP_Acc_Intep_HomePosition, MP_time_Intep_HomePosition ] = ...
                                          MotionPlanningOptimalSoultion_debugging(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                                  Mode_current, PosOri_current, q0q1q2_current, ...
                                                                                  NumIntepoPoints, Start_Time, Time_inteval, l1, l2);  
                                                                              
    % Position, Velocity, Acceleration Calcuation of joint angles                                                                                                       NumIntepoPoints, Start_Time, Time_inteval, l1, l2);                                      
      q0q1q2_Pos_mat = [ q0q1q2_Pos_mat; q0q1q2_P2P_Pos_Intep_HomePosition ];
      q0q1q2_Vel_mat = [ q0q1q2_Vel_mat; q0q1q2_P2P_Vel_Intep_HomePosition ];
      q0q1q2_Acc_mat = [ q0q1q2_Acc_mat; q0q1q2_P2P_Acc_Intep_HomePosition ];
    % Position, Velocity, Acceleration Calcuation of Moving Platform   
          MP_Pos_mat = [ MP_Pos_mat; MP_Pos_Intep_HomePosition ];
          MP_Vel_mat = [ MP_Vel_mat; MP_Vel_Intep_HomePosition ];
          MP_Acc_mat = [ MP_Acc_mat; MP_Acc_Intep_HomePosition ];
            Time_mat = [ Time_mat; MP_time_Intep_HomePosition ];
            
end
% --------------
toc

%% Save the value as '.mat' file
Len_q0q1q2_mat = length(q0q1q2_Pos_mat);
q11q12q14_q21q22q23 = [q0q1q2_Pos_mat(:,2), q0q1q2_Vel_mat(:,2), q0q1q2_Acc_mat(:,2),...
                       q0q1q2_Pos_mat(:,3), q0q1q2_Vel_mat(:,3), q0q1q2_Acc_mat(:,3),...
                       q0q1q2_Pos_mat(:,5), q0q1q2_Vel_mat(:,5), q0q1q2_Acc_mat(:,5),...
                       q0q1q2_Pos_mat(:,7), q0q1q2_Vel_mat(:,7), q0q1q2_Acc_mat(:,7),...
                       q0q1q2_Pos_mat(:,8), q0q1q2_Vel_mat(:,8), q0q1q2_Acc_mat(:,8),...
                       q0q1q2_Pos_mat(:,9), q0q1q2_Vel_mat(:,9), q0q1q2_Acc_mat(:,9),...
                       Time_mat(:,1)
                       ];
% save('q0q1q2_mat_Angle')

%% Check the correctness of the result by comparing the related adjunct values
LimitCheck_Redius = (360/NumIntepoPoints)*pi/180;
LimitCheck_Angle = 360/NumIntepoPoints;
for i_CC_row = 1: length(q0q1q2_Pos_mat) - 1% CorrectnessCheck
    delta_q0q1q2_mat = abs(q0q1q2_Pos_mat(i_CC_row + 1,:) - q0q1q2_Pos_mat(i_CC_row,:));
    for i_CC_colum = 1: length(delta_q0q1q2_mat)
        if delta_q0q1q2_mat(i_CC_colum) > LimitCheck_Redius
            errordlg('Input values are incorrect!, Please Check!','Check Value Error');
            error('Error. \n Output increment is large than %g degree, in row: %g, colum: %g.', LimitCheck_Angle, i_CC_row + 1, i_CC_colum)
        end
    end     
end
h = msgbox('Check Completed, Input values are correct!');
%% Plot joint Angles
%PlotAngleValue;

%% 3D Animation
for i = 1:length(q0q1q2_Pos_mat)-0  
    %========================== Animation ============================
    ReconbotANI(q0q1q2_Pos_mat(i,:));   
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

%clr_trail_CollisionPoints_button_press;
