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

l1 = 230.1390;
l2 = 147.7;

q0q1q2_HomePosition = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
p_0 = [0 0 208.879343162506 0 0 0, 0 0]; %  p = 2 * l2 * sin(pi/4)

deg = pi/180;

addpath(genpath(pwd)); % Enalbe all folders inside "Reconbot3Ddemo"
 
%InitHome   

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
    
    %% ========================= First-Planning for Singurlarity Judging================================ 
    %% Motion planning
    clc    
    for OnlyUsedforFolding_HomePos2SelectedEndPos = 1:1
        %=======================================
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
        %
        Mode_det_Jq_Jc_J_mat = [];
        Mode_det_Jq_Jc_J = [];
        Mode_det_Jq_Jc_J_HomePosition_mat = [];
        Mode_det_Jq_Jc_J_HomePosition = [];
        %
        PosOri_Output_mat = {};
        
        tic
        for IntepPointNum = 1 : NumTP
            
            if IntepPointNum == 1
                Mode = 5;
                Posture = [0 0 208.879343162506 0 0 0, 0 0];
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
                    q0q1q2_2RserialA1C1_x_0_y_Positive30 = [0 -3.14159265358979 0.139358669266875 3.14159265358979 -0.971371068617276 -3.14159265358979 -3.14159265358979 0.966880969134940 1.94661464276441 -0.603915357659957 -3.14159265358979];
                    q0q1q2_previous = [q0q1q2_2RserialA1C1_x_0_y_Positive30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
                elseif Mode_previous == 11 && Mode_current  == 5
                    q0q1q2_2RserialA2C2_x_0_y_Negative30 = [0 0 0.966880969134940 1.94661464276441 -0.603915357659957 0 0 0.139359146095574 3.14159265358979 -0.971371545445974 0];
                    q0q1q2_previous = [q0q1q2_2RserialA2C2_x_0_y_Negative30; q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:)];
                else
                    q0q1q2_previous = q0q1q2_Pos_mat(length(q0q1q2_Pos_mat(:,1)),:);
                end
            end
            q0q1q2_current = Mode_Pos_Ori_TrajPoints_cell{IntepPointNum + 1,1}{3};
            
            % Motion Planning and Optimal Soultion;
            [ PosOri_Output, q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, ...
                MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep, Mode_det_Jq_Jc_J ] = MotionPlanningTransitionModes(Mode_previous,PosOri_previous,q0q1q2_previous,...
                Mode_current, PosOri_current, q0q1q2_current, ...
                NumIntepoPoints, Start_Time, Time_inteval, l1, l2);
            % Moving Platform Position
            PosOri_Output_mat = [PosOri_Output_mat; PosOri_Output];
            
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
        
        %=======================================
        % Save the value as '.mat' file
        Len_q0q1q2_mat = length(q0q1q2_Pos_mat);
        q11q12q14_q21q22q23 = [ q0q1q2_Pos_mat(:,2), q0q1q2_Vel_mat(:,2), q0q1q2_Acc_mat(:,2),...
            q0q1q2_Pos_mat(:,3), q0q1q2_Vel_mat(:,3), q0q1q2_Acc_mat(:,3),...
            q0q1q2_Pos_mat(:,5), q0q1q2_Vel_mat(:,5), q0q1q2_Acc_mat(:,5),...
            q0q1q2_Pos_mat(:,7), q0q1q2_Vel_mat(:,7), q0q1q2_Acc_mat(:,7),...
            q0q1q2_Pos_mat(:,8), q0q1q2_Vel_mat(:,8), q0q1q2_Acc_mat(:,8),...
            q0q1q2_Pos_mat(:,9), q0q1q2_Vel_mat(:,9), q0q1q2_Acc_mat(:,9),...
            Time_mat(:,1), Mode_det_Jq_Jc_J_mat
            ];
        
        HomePos2SelectedEndPos_OutputData_Origin.JointSpace.PosOri_Output_mat = PosOri_Output_mat;
        HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat = q0q1q2_Pos_mat;
        HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Vel_mat = q0q1q2_Vel_mat;
        HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Acc_mat = q0q1q2_Acc_mat;
        HomePos2SelectedEndPos_OutputData_Origin.CartesianSpace.MP_Pos_mat = MP_Pos_mat;
        HomePos2SelectedEndPos_OutputData_Origin.CartesianSpace.MP_Vel_mat = MP_Vel_mat;
        HomePos2SelectedEndPos_OutputData_Origin.CartesianSpace.MP_Acc_mat = MP_Acc_mat;
        HomePos2SelectedEndPos_OutputData_Origin.Time_mat = Time_mat;
        HomePos2SelectedEndPos_OutputData_Origin.Mode_det_Jq_Jc_J_mat = Mode_det_Jq_Jc_J_mat;
        HomePos2SelectedEndPos_OutputData_Origin.q11q12q14_q21q22q23 = q11q12q14_q21q22q23;
        
        % =============== Assign Initial Values =============
        q0q1q2_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat;
        q11q12q14_q21q22q23_NewAdjust = HomePos2SelectedEndPos_OutputData_Origin.q11q12q14_q21q22q23;
        MP_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_Origin.CartesianSpace.MP_Pos_mat;
        Time_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_Origin.Time_mat;
        
        %% =============== Plot joint Angles and Check the correntness of Joint Velocity Limitation ==================
        for OnlyUsedforFolding_ValueCheck_PlotValue = 1:1
            
            % =============== Plot joint Angles ==============
            step = 20;
            PlotAngleValue;
            
            %=============== Check the correctness of the result by comparing the related adjunct values ===============
            VelocityLimitCheck_Redius = 3.0; % Maximum Speed without Load: 6.282 rad/s;
            VelocitLimitCheck_Angle = 180; % degree/s;
            for i_CC_row = 1: length(q0q1q2_Pos_mat_NewAdjust) - 1% CorrectnessCheck
                
                for i_CC_colum = 1: 6
                    i_colum = (i_CC_colum - 1) * 3 + 1;
                    if q11q12q14_q21q22q23_NewAdjust(i_colum) > VelocityLimitCheck_Redius
                        errordlg('Velocity Exceed Limits!, Please Check!','Check Value Error');
                        error('Error. \n Output Velocity is large than %g degree/s, in row: %g, colum: %g.', LimitCheck_Angle, i_CC_row + 1, i_colum)
                    end
                end
                
            end
            h = msgbox('Check Completed, Velocity of Motors are in Limit!');
            
            %  =============================== 3D Animation =================================
            %HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(:,1) = HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(:,2);
            for i = 1:length(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat)
                ReconbotANI(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(i,:));
            end
            
        end
        
    end
        
    
    %% clr_trail_CollisionPoints_button_press;
    %step = 100;
    %ReconbotANI(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(step,:));
    
    
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
    
    %% in World frame    
    %----------------- Axis angle -----------------------
    EulerAngle_Output = HomePos2SelectedEndPos_OutputData_Origin.CartesianSpace.MP_Pos_mat(step,4:6);
    OperationalMode = HomePos2SelectedEndPos_OutputData_Origin.Mode_det_Jq_Jc_J_mat(step,1);
    %-- m=vrrotvec2mat(r):Convert rotation from axis-angle to matrix representation--
    if OperationalMode <= 5
        RotationMatrix = rotz(MP_Pos_mat(step,6)*180/pi);
    else
        RotationMatrix = eul2rotm(EulerAngle_Output);
    end
    %----------------- plot xyz axes of Moving Platform --------------
    x_axis = [50 0 0];
    y_axis = [0 50 0];
    z_axis = [0 0 50];
    OP= [0 0 0];
    xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
    xyz = (RotationMatrix * rotz(90) * xyz')' + (rotz(90) * (ones(length(xyz(:,1)),1) * MP_Pos_mat(step,1:3))')' + ones(length(xyz(:,1)),1) * Displacement;
    j = 1:2;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2); hold on
    j = 3:4;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2); hold on
    j = 5:6;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2); hold on
    title('Moving Platform Trajectory')
    grid on
    axis equal
    %--------------------------------------------------------------
    
    %%
    %----------------- plot xyz axes of Base Platform --------------
    Displacement = [250,250,83.5+60.44+(45.5-22)];
    MP_Pos = (rotz(90) * MP_Pos_mat(:,1:3)')' + ones(length(MP_Pos_mat(:,1)),1) * Displacement;
    
    plot3(MP_Pos(:,1),MP_Pos(:,2),MP_Pos(:,3),'k-', 'Linewidth', 2); hold on
    %----------------- plot xyz axes of base point --------------
    x_axis = [50 0 0];
    y_axis = [0 50 0];
    z_axis = [0 0 50];
    OP= [0 0 0];
    xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
    xyz = (rotz(90) * xyz')' + ones(length(xyz(:,1)),1) * Displacement;
    j = 1:2;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2); hold on
    j = 3:4;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2); hold on
    j = 5:6;
    plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2); hold on
    title('Moving Platform Trajectory')
    grid on
    axis equal
    %--------------------------------------------------------------
    
   
    %------------------------------------------------------------
    
    %%
    
end