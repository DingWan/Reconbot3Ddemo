%%--------------------2-RER Mode selection-----------------------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 15 Nov 2016.

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

l1 = 0.2301390;
l2 = 0.1477;

q0q1q2_HomePosition = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
p_0 = [0 0 0.208879343162506 0 0 0, 0 0]; %  p = 2 * l2 * sin(pi/4)

deg = pi/180;

addpath(genpath(pwd)); % Enalbe all folders inside "Reconbot3Ddemo"
% InitHome   

%% Go to HomePosition
% Construct a questdlg with three options
choice = questdlg('Is Reconbot in Home Position?', ...
	'Home Position', ...
	'No,Go HomePosition','Yes,Please Continue','Stop Calculation');
% Handle response
switch choice
    case 'No,Go HomePosition'
        disp([choice ' Detecting Motor Encoders...'])
        HomePosition = 1;
        RandomPosition2HomePosition;
    case 'Yes,Please Continue'
        disp('Inputing Trajectory Points...')
        HomePosition = 2;
    case 'Stop Calculation'
        disp('Calculation Stopped...')
        HomePosition = 0;        
end

if HomePosition == 2
    %% Select Mode + Posture
    SelectNumberOfTrajectoryPoints;
    
    %------ Single Mode ----------
    % load('q0q1q2_3T2R.mat')
    
    %% Motion planning
    clc
    % Intepotation Points and Time
    NumIntepoPoints = 50;
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
    Mode_det_Jq_Jc_J_mat = [];
    Mode_det_Jq_Jc_J = [];
    Mode_det_Jq_Jc_J_HomePosition_mat = [];
    Mode_det_Jq_Jc_J_HomePosition = [];
    %
    
    tic
    for IntepPointNum = 1 : NumTP
        
        if IntepPointNum == 1
            Mode = 5;
            Posture = [0 0 0.208879343162506 0 0 0, 0 0];
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
            if Mode_previous == 10 && Mode_current  == 5
                q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 -3.14159265358979 0.139358669266875 3.14159265358979 -0.971371068617276 -3.14159265358979 0 0.966880969134940 1.94661464276441 -0.603915357659957 0];
                q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
            elseif Mode_previous == 11 && Mode_current  == 5
                q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0 0 0.966880969134940 1.94661464276441 -0.603915357659957 0 -3.14159265358979 0.139358669266875 3.14159265358979 -0.971371068617276 -3.14159265358979];
                q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
            else
                q0q1q2_previous = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:);
            end
        end
        q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{3};
        
        % Motion Planning and Optimal Soultion;
        [ q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, ...
            MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep, Mode_det_Jq_Jc_J ] = MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
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
        % Mode, Jacobian of Jq and J
        Mode_det_Jq_Jc_J_mat = [Mode_det_Jq_Jc_J_mat; Mode_det_Jq_Jc_J];
        
        Start_Time = Time_mat(length(Time_mat(:,1)),1);
        
    end
    toc
    
    IntepPointNum = IntepPointNum + 2;
    %% Last step for returnning to HomePosition
    tic
    %---------------
    for OnlyUsedforFoldingThisPart = 1:1
        Mode_Pos_Ori_TrajPoints_cell{IntepPointNum} = { 5, {0 0 0.208879343162506 0 [] [], 0 ,0},q0q1q2_HomePosition};
        % First step: Calculate the next step and get the second row values of q11 and q21 after
        % interpotation, and assign to the previous step.0
        % Assgin Input value
        Mode_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{1};
        Mode_current  = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{1};
        
        PosOri_previous = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum - 1}{2};
        PosOri_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{2};
        
        if Mode_previous == 10 && Mode_current  == 5
            q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 -3.14159265358979 0.139358669266875 3.14159265358979 -0.971371068617276 -3.14159265358979 0 0.966880969134940 1.94661464276441 -0.603915357659957 0];
            q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
        elseif Mode_previous == 11 && Mode_current  == 5
            q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0 0 0.966880969134940 1.94661464276441 -0.603915357659957 0 -3.14159265358979 0.139358669266875 3.14159265358979 -0.971371068617276 -3.14159265358979];
            q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
        else
            q0q1q2_previous = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:);
        end
        q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum}{3};
        
        % Motion Planning and Optimal Soultion;
        [ q0q1q2_P2P_Pos_Intep_HomePosition, q0q1q2_P2P_Vel_Intep_HomePosition ,q0q1q2_P2P_Acc_Intep_HomePosition, ...
            MP_Pos_Intep_HomePosition, MP_Vel_Intep_HomePosition, MP_Acc_Intep_HomePosition, MP_time_Intep_HomePosition, Mode_det_Jq_Jc_J_HomePosition ] = ...
                                                                            MotionPlanningOptimalSoultion(Mode_previous,PosOri_previous,q0q1q2_previous,...
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
        % Mode, Jacobian of Jq and J
        Mode_det_Jq_Jc_J_mat = [Mode_det_Jq_Jc_J_mat; Mode_det_Jq_Jc_J_HomePosition];
        
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
    
    %% Plot joint Angles
    PlotAngleValue;
    
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
    
    %% 3D Animation
    for i = 41:length(q0q1q2_Pos_mat)-40
        %========================== Animation ============================
        ReconbotANI(q0q1q2_Pos_mat(i,:));
        % ReconbotANI(q0q1q2_Pos_mat(51,:));
        %     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
        %     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
        %============================ End ================================
    end
    
    
    %% Moving Platform Trajectory
    % Displacement = [250,250,167.4400] / 1000;
    % for i = 1:length(q0q1q2_Pos_mat(:,1))
    %     Displacement_mat(i,:) = Displacement;
    % end
    % p = q0q1q2_Pos_mat(:,1:3) + Displacement_mat;
    % handles = getappdata(0,'patch_h');
    % Tr = handles(12);
    % %============= Center Point of Moving Platform =================
    % set(Tr,'xdata',p(1),'ydata',p(2),'zdata',p(3));
    % %============================ End ==============================
    
    %----------------- plot xyz axes of base point --------------
    Displacement = [250,250,83.5+60.44+(45.5-22)] / 1000 ;
    x_axis = [40 0 0] / 1000 + Displacement;
    y_axis = [0 40 0] / 1000 + Displacement;
    z_axis = [0 0 40] / 1000 + Displacement;
    OP= [0 0 0] + Displacement;
    xyz = (rotz(-90) * [OP;x_axis;OP;y_axis;OP;z_axis]')';
    j = 1:2;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2); hold on
    j = 3:4;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2); hold on
    j = 5:6;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2); hold on
    %------------------------------------------------------------
    %------------------------------------------------------------
    
    %%
    % clr_trail_CollisionPoints_button_press;
end