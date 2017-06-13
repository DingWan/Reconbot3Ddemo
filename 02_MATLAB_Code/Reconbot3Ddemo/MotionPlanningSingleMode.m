function [ PosOri_Output_Intep, q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep, Mode_det_Jq_Jc_J_Intep ] = ...
    MotionPlanningSingleMode(Mode_previous, PosOri_previous, q0q1q2_previous_trajpoint,...
    Mode_current,PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Start_Time, Time_inteval, l1, l2)
% Motion planning and Optimal Solution
%
%      Mode_current_initial: The Selected mode in Main_ModeSelection at current step
%     Mode_previous_initial: The Selected mode in Main_ModeSelection at previous step
%             Mode_previous: The mode (1~12) of previous one step;
%          Posture_previous: The Posture (Position + Orientation + q11 + q21) of previous one step;
%           q0q1q2_previous: The joints value [q0, q11~q15, q21~q25] of previous one step;
%              Mode_current: The mode (1~12) of previous one step;
%           Posture_current: The Posture (Position + Orientation + q11 + q21) of previous one step;
%            q0q1q2_current: The joints value [q0, q11~q15, q21~q25] of previous one step;
%            NumIntepoPoint: Current intepotation number
%                Start_Time: the Intepotation start Time for current step
%              Time_inteval: the time interval for intepotation on each i step,  i = 1:length(MPOTP_cell)
%
%                            MPOTP_cell: Motion planning of Trajectory Points in cell mode with infos: {mode, {position, orientation, q11, q21}}
%        Self_adjustment_Enable_Disable: The correct order of differernt value with ' = 1/2/3/0'
%                                        1: The first to second mode, q11/q12 needs to be adjust
%                                        2: Random Mode to HomePosition, q11/q12 needs to be adjust
%                                        3: After Random Mode to mode, q11/q12 needs to be adjust
%                                        0: No needs to be adjust
%  Self_adjustment_Enable_Disable_Array: All possible self_adjustment assigned in each step
%
%                 Pos_Intep: Position intepotation for each i step
%                 Vel_Intep: Velocity intepotation for each i step
%                 Acc_Intep: Acceleration intepotation for each i step
%                time_Intep: Intepotation Time for each i step
% %
%              MP_Pos_Intep: Position intepotation for all steps of Moving Platform
%              MP_Vel_Intep: Velocity intepotation for all steps of Moving Platform
%              MP_Acc_Intep: Acceleration intepotation for all steps of Moving Platform
%             MP_time_Intep: Intepotation Time for all steps of Moving Platform
% %
%      q0q1q2_P2P_Pos_Intep: Angle intepotation for all steps of all input joint angles
%      q0q1q2_P2P_Vel_Intep: Angular velocity intepotation for all steps of all input joints
%      q0q1q2_P2P_Acc_Intep: Angular acceleration intepotation for all steps of all input joints
%     q0q1q2_P2P_time_Intep: Intepotation Time for all steps of all input joints 
L1 = l1;
L2 = l2;
%% Transition Strategy
[ MPOTP_cell, Self_adjustment_Enable_Disable_Array ] = TransitionStrategy(Mode_previous,PosOri_previous, q0q1q2_previous_trajpoint, Mode_current,PosOri_current, l1, l2);

%%
Pos_Intep = [];
Vel_Intep = []; 
Acc_Intep = []; 
time_Intep = [];
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
det_Jq_Normalized = [];
det_J_Normalized = [];
%
col = []; % Number and position of 'Self_adjustment_Enable_Disable == 1'
col_Self_adjustment = [];
q0q1q2_P2P_Pos_Intep = [];
Mode_previous_initial = Mode_previous;
PosOri_previous_initial = PosOri_previous;
Mode_current_initial = Mode_current;
PosOri_current_initial = PosOri_current;

for i = 1:length(MPOTP_cell)
    
    %% Assign value to "Self_adjustment_Enable_Disable == 0/1/2/3"
    Self_adjustment_Enable_Disable = Self_adjustment_Enable_Disable_Array(i); 
    % Here uses to get the first 1 for self-adjustment
    [~,col_Self_adjustment] = find(Self_adjustment_Enable_Disable_Array == 1);
    if isempty(col_Self_adjustment)
        col_Self_adjustment = 0;
    end
    
    %% Assgin "Mode_Previous, Mode_Current, PosOri_Current, PosOri_Previous"
    if Self_adjustment_Enable_Disable == 0
        Mode_previous;
        PosOri_previous;
        Mode_current = MPOTP_cell{i}{1};
        PosOri_current = MPOTP_cell{i}{2};
        Mode = Mode_current;
    end
    
    %% Intepotation and Trajectory Planning
    for OnlyUsedforFoldingThisPart_TrajectoryPlanning = 1:1
        Time = [(i-1) * Time_inteval, i * Time_inteval] + Start_Time * [1 1];
        
        [ Pos_Intep, Vel_Intep, Acc_Intep, time_Intep ] =  ...
            IntepotationP2P(Mode, PosOri_previous,q0q1q2_previous_trajpoint, PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Time, l1, l2);
        
        % Position, Velocity, Acceleration Calcuation
        if length(Pos_Intep(:,1)) == 6
            Pos_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(7:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)]; 
        elseif length(Pos_Intep(:,1)) == 7
            Pos_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(8:10,:) = [ zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];              
        elseif length(Pos_Intep(:,1)) == 8
            Pos_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Vel_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];
            Acc_Intep(9:10,:) = [zeros(1,NumIntepoPoints); zeros(1,NumIntepoPoints)];    
        end
        MP_Pos_Intep = [MP_Pos_Intep; Pos_Intep']; %MP: moving Platform
        MP_Vel_Intep = [MP_Vel_Intep; Vel_Intep'];
        MP_Acc_Intep = [MP_Acc_Intep; Acc_Intep'];
        MP_time_Intep = [MP_time_Intep; time_Intep'];
    end
    
    %% Iterative IK solution of all intepolation points
    for j = 1:NumIntepoPoints
        %
        if Mode == Mode_current
            PosOri_Effect = PosOri_current;
        elseif Mode == Mode_previous
            PosOri_Effect = PosOri_previous;
        end
        PosOri = {};
        for k = 1:length(PosOri_Effect)
            if isempty(PosOri_Effect{k}) ~= 1
                PosOri{k} = Pos_Intep(k,j);
            else
                PosOri{k} = [];
            end
        end

        for OnlyUsedforFoldingThisPart_TrajectoryPlanning = 1:1
            % We assign the q11 and q21 with previous one step value
            if i == 1
                if j == 1
                    % only used for the first step from HomePosition to other mode
                    q0 = q0q1q2_previous_trajpoint(1);
                    q11 = q0q1q2_previous_trajpoint(2);
                    q21 = q0q1q2_previous_trajpoint(7);
                    q0q1q2_OptimalRow(1,:) = q0q1q2_previous_trajpoint;
                else
                    %if self-adjustment exists
                    q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
                    if Mode == 3
                        q11 = PosOri{7};
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    elseif Mode == 4
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = PosOri{8};
                    elseif Mode == 5
                        if Mode_previous == 5 && Mode_previous == Mode_current
                            q11 = PosOri{7};
                            q21 = PosOri{8};
                        else
                            q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                            q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                        end
                    else
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    end
                end            
            end
        end
        
        % IK solution
        
        [ p, ~, ~, q1q2, ~ ] = IK(Mode, PosOri, q0, q11, q21, q0q1q2_OptimalRow(length(q0q1q2_OptimalRow(:,1)),:), l1, l2);
        q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];
           
        % Only used for judge the transtion needs Mode= 6 and PosOri = {0, +/-0.030, [], [], [], [], q12/q22}
        if Start_Time == 40   
            STOP = 1;
        end
        
            if Mode == 6 && PosOri{1} == 0
                if abs(PosOri{2} - 0.030) < 1e-12 || abs(PosOri{2} + 0.030) < 1e-12
                    if j == 1
                        q0q1q2_CurrentStep = q0q1q2_previous_trajpoint;
                    else
                        q0q1q2_CurrentStep = q0q1q2_current_trajpoint;
                    end
                end 
            elseif (Mode == 10 || Mode == 11) && abs(PosOri{1}) < 1e-12 && abs(PosOri{2}) < 1e-12
                if j == 1
                    q0q1q2_CurrentStep = q0q1q2_previous_trajpoint;
                else
                    q0q1q2_CurrentStep = q0q1q2_current_trajpoint;
                end
            end
            
        % Optimal Joints Solution
        OptimalJointsSolution;
        q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j,:) = q0q1q2_Optimal_SingleRow;  
        %
        PosOri_Output{NumIntepoPoints*(i-1)+j,1} = PosOri;      
        
        % ==================  Jacobian Matrix  ====================
        q1q2_Optimal = q0q1q2_Optimal_SingleRow(:,2:11);
        q11 = q1q2_Optimal(1); q12 = q1q2_Optimal(2); q13 = q1q2_Optimal(3); q14 = q1q2_Optimal(4); q15 = q1q2_Optimal(5);
        q21 = q1q2_Optimal(6); q22 = q1q2_Optimal(7); q23 = q1q2_Optimal(8); q24 = q1q2_Optimal(9); q25 = q1q2_Optimal(10);
        
        if  Mode == 1
            Enable_Mode_JacoMat = 1;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = det(Jq1_Ob_3T2R); 
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = 0;%det(J_Ob_3T2R);
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 0;
        elseif  Mode == 2
            Enable_Mode_JacoMat = 2;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = det(Jq1_Ob_3T1R); 
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = det(J_Ob_3T1R);
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = det(Jc_Ob_3T1R);
        elseif  Mode == 5
            Enable_Mode_JacoMat = 5;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R)-------
            det_Jq(j,:) = 0; % scaled
            %-------det_J_Ob_3T1R(NumIntepoPoints*(i-1)+j,:) = det(J_Ob_3T1R)----
            det_J(j,:) = 0;
            % ------Jc_Ob_3T1R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = det(Jc_Ob_HomePosition);
        elseif Mode == 6 || Mode == 7 || Mode == 8 || Mode == 9
            Enable_Mode_JacoMat = 6;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2T2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2T2Rsixbar); % scaled
            %-------det_J_Ob_2T2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2T2Rsixbar);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        elseif Mode == 10
            Enable_Mode_JacoMat = 10;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2RSerialChainA1C1); % scaled
            %-------det_J_Ob_2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2RSerialChainA1C1);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        elseif Mode == 11
            Enable_Mode_JacoMat = 11;
            UnifiedJacobianMatrix_ScrewTheory;
            %-------det_Jq2_Ob_2T2R = det(Jq2_Ob_2Rsixbar)-------
            det_Jq(j,:) = det(Jq2_Ob_2RSerialChainA2C2); % scaled
            %-------det_J_Ob_2R = det(J_Ob_2T2R)-------
            det_J(j,:) = det(J_Ob_2RSerialChainA2C2);
            % ------Jc_Ob_2T2R------
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        else
            det_Jq(j,:) = 1;
            det_J(j,:) = 1;
            det_Jc(NumIntepoPoints*(i-1)+j,:) = 1;
        end
        
        if abs(det_J(j,:)) < 1e-12
            det_J(j,:) = 0;
        end
                
        Mode_Ideal(NumIntepoPoints*(i-1)+j,:) = Mode;

        %------ Show Center point of Moving Platform -------
        %Displacement = [250,250,165.88];
        %p_Base = p(1:3) + Displacement;
        %plot3(p_Base(1),p_Base(2),p_Base(3),'r.');
    end    
    
    %------------------------------------------------------ 
    % det_Jq_Normalized =  det_Jq / max(abs(det_Jq))
    [row,~] = find(abs(det_Jq) == max(abs(det_Jq)));
    if max(abs(det_Jq)) == 0
        det_Jq_Normalized = [det_Jq_Normalized; det_Jq];
    else
        det_Jq_Normalized = [det_Jq_Normalized; det_Jq/abs(det_Jq(row(1)))];
    end
    % det_J_Normalized =  det_J / max(abs(det_J))
    [row,~] = find(abs(det_J) == max(abs(det_J)));
    if max(abs(det_J)) == 0
        det_J_Normalized = [det_J_Normalized; det_J];
    else
        det_J_Normalized = [det_J_Normalized; det_J/abs(det_J(row(1)))];
    end    
    
    %------------------------------------------------------ 
    % Output current acutall modes
    % We define:
    for j = 1:NumIntepoPoints
        if ( Mode ~= 5 && abs(det_Jq_Normalized(NumIntepoPoints*(i-1)+j,:) ) < 0.2) ||...
                ( Mode ~= 1 && abs(det_Jc(NumIntepoPoints*(i-1)+j,:)) < 0.2 )
            Mode_Actual(NumIntepoPoints*(i-1)+j,:) = 0;
        else
            Mode_Actual(NumIntepoPoints*(i-1)+j,:) = Mode;
        end
    end
    % ===========================  End  ============================
    
end

%% Output current all modes, Jacobian of Jq and J
% Mode_det_Jq_Jc_J = Mode_Actual;
Mode_det_Jq_Jc_J = [Mode_Ideal, Mode_Actual, det_Jq_Normalized, det_Jc, det_J_Normalized];% det_J
Mode_det_Jq_Jc_J_Intep = Mode_det_Jq_Jc_J;

%% Assign the correct order of differernt value of 'Self_adjustment_Enable_Disable = 1/2/3/0'
q0q1q2_P2P_Pos_Intep = q0q1q2_OptimalRow;
PosOri_Output_Intep = PosOri_Output;    
    
%% Position, Velocity, Acceleration Calcuation of joint angles
% Velocity Calcuation
for i = 1:length(MPOTP_cell)
    for j = 1:NumIntepoPoints
        if j == 1 || j == NumIntepoPoints
            q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
        else
            q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:) = ...
            ( q0q1q2_P2P_Pos_Intep((i-1)*NumIntepoPoints + j + 1,:) - q0q1q2_P2P_Pos_Intep((i-1)*NumIntepoPoints + j,:) ) / ...
            (       MP_time_Intep((i-1)*NumIntepoPoints + j + 1, 1) - MP_time_Intep((i-1)*NumIntepoPoints + j, 1)       ); 
        end
    end
end

% Acceleration Calcuation
for i = 1:length(MPOTP_cell)
    for j = 1:NumIntepoPoints
        if j == 1 || j == NumIntepoPoints
            q0q1q2_P2P_Acc_Intep((i-1)*NumIntepoPoints + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
        else
            q0q1q2_P2P_Acc_Intep((i-1)*NumIntepoPoints + j,:) = ...
            ( q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j + 1,:) - q0q1q2_P2P_Vel_Intep((i-1)*NumIntepoPoints + j,:)  ) / ...
            (       MP_time_Intep((i-1)*NumIntepoPoints + j + 1, 1) - MP_time_Intep((i-1)*NumIntepoPoints + j, 1)        ); 
        end
    end
end

end
