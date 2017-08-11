
%     for i=1:4
%         axes{i}=['axes',num2str(i)];
%     end
% 
%    for i=1:4
%        cla(handles.(axes{i}));
%    end 

    Global_Para;
    NumIntepoPoints = 20;
    Start_Time = 0;
    Time_inteval = 5;

%% ======================== Re-Planning According to Singurlarity=========================================
    %Detect Mode switch and replan according the 'Start and End velocities are Zero: V_start = V_end = 0'
    
    for OnlyUsedforFolding_RePlan = 1:1
    NumIntepoPoints_DetailedPlanning = 20;
    
    Mode_det_Jq_Jc_J_mat=HomePos2SelectedEndPos_OutputData_Origin.Mode_det_Jq_Jc_J_mat;
    Mode_Ideal = Mode_det_Jq_Jc_J_mat(:,1);
    Mode_Actual = Mode_det_Jq_Jc_J_mat(:,2);
    %
    PosOri_Output_mat_NewAdjust = [];
    %
    q0q1q2_Pos_mat_NewAdjust = [];
    q0q1q2_Vel_mat_NewAdjust = [];
    q0q1q2_Acc_mat_NewAdjust = [];
    %
    MP_Pos_mat_NewAdjust = [];
    MP_Vel_mat_NewAdjust = [];
    MP_Acc_mat_NewAdjust = [];
    Time_mat_NewAdjust = [];
    %
    Mode_det_Jq_Jc_J_mat_NewAdjust = [];
    %
    Start_Time_NewAdjust = 0;
    
    tic
    for i = 1:length(q0q1q2_Pos_mat)/NumIntepoPoints
        
        % Here below is to judge the boundary of mode change;
        % 1. Both ends are also the first and last row of 'Row_ModeBoundary';
        % 2. The switch of equal and inequal is a boundary sign
        % 3. So, this judge can cover the case like: [0 1 1 1 0 0 1 1 1 0 0 0 1 0 0 1 1 1 0]
        %    The boundary vector should be: Row_ModeBoundary = [2 4 7 9 13 16 18]
        Num_BoundaryPoints = 0;
        Row_ModeBoundary = [];
        Temp_Row_ModeBoundary = [];
        for j =  1 : NumIntepoPoints 
            if j < NumIntepoPoints
                ideal_pre =  Mode_Ideal(NumIntepoPoints*(i-1)+j);
                Actual_pre =  Mode_Actual(NumIntepoPoints*(i-1)+j);
                ideal_Next =  Mode_Ideal(NumIntepoPoints*(i-1)+j+1);
                Actual_Next =  Mode_Actual(NumIntepoPoints*(i-1)+j+1);
                if ideal_pre == Actual_pre && ideal_Next ~= Actual_Next
                    Num_BoundaryPoints = Num_BoundaryPoints + 1;
                    Row_ModeBoundary(Num_BoundaryPoints) = NumIntepoPoints*(i-1) + j;
                elseif ideal_pre ~= Actual_pre && ideal_Next == Actual_Next
                    Num_BoundaryPoints = Num_BoundaryPoints + 1;
                    Row_ModeBoundary(Num_BoundaryPoints) = NumIntepoPoints*(i-1) + j + 1;
                end
            else
                Temp_Row_ModeBoundary = Row_ModeBoundary;
                
                Row_ModeBoundary(1) = NumIntepoPoints*(i-1)+1;
                if isempty(Temp_Row_ModeBoundary) ~= 1
                    Row_ModeBoundary(2:length(Temp_Row_ModeBoundary) + 1) = Temp_Row_ModeBoundary;                    
                    if Temp_Row_ModeBoundary(length(Temp_Row_ModeBoundary)) ~= NumIntepoPoints*i
                        Row_ModeBoundary(length(Temp_Row_ModeBoundary) + 2) = NumIntepoPoints*i;
                    end
                else
                    Row_ModeBoundary(2) = NumIntepoPoints*i;
                end
            end
        end
        
        % Here blow is to recalculate the changed steps and Adjust time;
        for NumRecalcTrajPoints = 1 : length(Row_ModeBoundary) - 1
                
            % Assgin Input value
            % Previous
            Mode_previous = Mode_Ideal( Row_ModeBoundary(NumRecalcTrajPoints) );%Mode_Pos_Ori_TrajPoints_cell
            PosOri_previous = PosOri_Output_mat{ Row_ModeBoundary(NumRecalcTrajPoints) };
            q0q1q2_previous = q0q1q2_Pos_mat(Row_ModeBoundary(NumRecalcTrajPoints),:);
            % Current
            Mode_current  = Mode_Ideal( Row_ModeBoundary(NumRecalcTrajPoints + 1) );
            PosOri_current = PosOri_Output_mat{ Row_ModeBoundary(NumRecalcTrajPoints + 1) };
            q0q1q2_current = q0q1q2_Pos_mat(Row_ModeBoundary(NumRecalcTrajPoints + 1),:);

            % Motion Planning and Optimal Soultion;
            [ PosOri_Output, q0q1q2_P2P_Pos_Intep, q0q1q2_P2P_Vel_Intep ,q0q1q2_P2P_Acc_Intep, ...
                MP_Pos_Intep, MP_Vel_Intep, MP_Acc_Intep, MP_time_Intep, Mode_det_Jq_Jc_J ] = MotionPlanningSingleMode(Mode_previous,PosOri_previous,q0q1q2_previous,...
                                                                                                                            Mode_current, PosOri_current, q0q1q2_current, ...
                                                                                                                            NumIntepoPoints_DetailedPlanning, Start_Time_NewAdjust, Time_inteval, l1, l2);

            % Moving Platform Position
            PosOri_Output_mat_NewAdjust = [PosOri_Output_mat_NewAdjust; PosOri_Output];
            % Position, Velocity, Acceleration Calcuation of joint angles
            q0q1q2_Pos_mat_NewAdjust = [q0q1q2_Pos_mat_NewAdjust; q0q1q2_P2P_Pos_Intep];
            q0q1q2_Vel_mat_NewAdjust = [q0q1q2_Vel_mat_NewAdjust; q0q1q2_P2P_Vel_Intep];
            q0q1q2_Acc_mat_NewAdjust = [q0q1q2_Acc_mat_NewAdjust; q0q1q2_P2P_Acc_Intep];            
            % Position, Velocity, Acceleration Calcuation of Moving Platform
            MP_Pos_mat_NewAdjust = [ MP_Pos_mat_NewAdjust; MP_Pos_Intep ];
            MP_Vel_mat_NewAdjust = [ MP_Vel_mat_NewAdjust; MP_Vel_Intep ];
            MP_Acc_mat_NewAdjust = [ MP_Acc_mat_NewAdjust; MP_Acc_Intep ];
            Time_mat_NewAdjust = [ Time_mat_NewAdjust; MP_time_Intep ];
            % Mode, Jacobian of Jq and J
            Mode_det_Jq_Jc_J(:,2) = Mode_Actual( Row_ModeBoundary(NumRecalcTrajPoints) ) * ones(NumIntepoPoints_DetailedPlanning, 1);
            Mode_det_Jq_Jc_J_mat_NewAdjust = [Mode_det_Jq_Jc_J_mat_NewAdjust; Mode_det_Jq_Jc_J];
            %
            Start_Time_NewAdjust = Time_mat_NewAdjust(length(Time_mat_NewAdjust(:,1)),1);
                
        end
    end
    toc
        
    % Save the value as '.mat' file
    Len_q0q1q2_mat = length(q0q1q2_Pos_mat);
    q11q12q14_q21q22q23_NewAdjust = [ q0q1q2_Pos_mat_NewAdjust(:,2), q0q1q2_Vel_mat_NewAdjust(:,2), q0q1q2_Acc_mat_NewAdjust(:,2),...
                                        q0q1q2_Pos_mat_NewAdjust(:,3), q0q1q2_Vel_mat_NewAdjust(:,3), q0q1q2_Acc_mat_NewAdjust(:,3),...
                                        q0q1q2_Pos_mat_NewAdjust(:,5), q0q1q2_Vel_mat_NewAdjust(:,5), q0q1q2_Acc_mat_NewAdjust(:,5),...
                                        q0q1q2_Pos_mat_NewAdjust(:,7), q0q1q2_Vel_mat_NewAdjust(:,7), q0q1q2_Acc_mat_NewAdjust(:,7),...
                                        q0q1q2_Pos_mat_NewAdjust(:,8), q0q1q2_Vel_mat_NewAdjust(:,8), q0q1q2_Acc_mat_NewAdjust(:,8),...
                                        q0q1q2_Pos_mat_NewAdjust(:,9), q0q1q2_Vel_mat_NewAdjust(:,9), q0q1q2_Acc_mat_NewAdjust(:,9),...
                                        Time_mat_NewAdjust(:,1), Mode_det_Jq_Jc_J_mat_NewAdjust
                                      ];
    
    end                   
                      
    HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.PosOri_Output_mat = PosOri_Output_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat = q0q1q2_Pos_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Vel_mat = q0q1q2_Vel_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Acc_mat = q0q1q2_Acc_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Pos_mat = MP_Pos_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Vel_mat = MP_Vel_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Acc_mat = MP_Acc_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.Time_mat = Time_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.Mode_det_Jq_Jc_J_mat = Mode_det_Jq_Jc_J_mat_NewAdjust;
    HomePos2SelectedEndPos_OutputData_RePlan.q11q12q14_q21q22q23 = q11q12q14_q21q22q23_NewAdjust;    
    
    % =============== Assign Initial Values =============
         q0q1q2_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat;
    q11q12q14_q21q22q23_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.q11q12q14_q21q22q23;
             MP_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Pos_mat;
               Time_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.Time_mat;
               
               
    %% =============== Plot joint Angles and Check the correntness of Joint Velocity Limitation ==================
    for OnlyUsedforFolding_ValueCheck_PlotValue = 1:1
        
        % =============== Plot joint Angles ==============
        PlotAngle_NewAdjust;
        
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
       % h = msgbox('Check Completed, Velocity of Motors are in Limit!');
    end
    
%      %  =============================== 3D Animation =================================
%         %SelectedEndPos2HomePos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,1) = SelectedEndPos2HomePos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,2);
%         for i = 1:length(SelectedEndPos2HomePos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat)
%             ReconbotANI(SelectedEndPos2HomePos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(i,:));
%         end
        
    
    