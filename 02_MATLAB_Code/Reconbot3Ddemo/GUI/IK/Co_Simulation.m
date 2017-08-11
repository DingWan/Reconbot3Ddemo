 
Global_Para;

%% ================================== Co-Simulation ==================================
    for OnlyUsedforFolding_CoSim = 1:1
        h = msgbox('Co_Simulation Calculation Completed, 1. Set Co_Simulation as Current Folder, 2. Run RCB_CoSim_FullMode_Output.slx');
        
        Co_Simulation_Enable = 1;
        
        LenHome2SelectedEndPos = length(HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat);
        if Co_Simulation_Enable == 1
            
             q0_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,1);
            q11_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,2);
            q12_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,3)-pi/4*ones(LenHome2SelectedEndPos,1);
            q14_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,5)+pi/4*ones(LenHome2SelectedEndPos,1);
            q21_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,7);
            q22_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,8)-pi/4*ones(LenHome2SelectedEndPos,1);
            q23_CoSim = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,9)-pi/2*ones(LenHome2SelectedEndPos,1);
            
            n = LenHome2SelectedEndPos;
            Slide = 0 * ones(n,1);
            for i = 1:n
                LeftArmAngle(i,1) = pi/6 * sin(i*pi/n);
                RightArmAngle(i,1) = pi/6 * sin(i*pi/n);
            end
            
            q0q1q2SlideLeftRightArm = [q0_CoSim, q11_CoSim, q12_CoSim, q14_CoSim, q21_CoSim, q22_CoSim, q23_CoSim, LeftArmAngle, RightArmAngle, Slide];% * 180/pi;
            q0q1q2SlideLeftRightArm_time = [HomePos2SelectedEndPos_OutputData_RePlan.Time_mat, q0q1q2SlideLeftRightArm];
            
            % 3D Animation
            for i = 1:length(q0q1q2_Pos_mat)- 0
                %ReconbotANI(q0q1q2_Pos_mat(i,:));
            end
            
            % Co-Simulation launch
            RCB_CoSim;
            
        end
    end    
       
    
    %%
    %clr_trail_CollisionPoints_button_press;
    
        %% 3D Animation
    %q0q1q2_Pos_mat(:,1) = q0q1q2_Pos_mat(:,2);
    for i = 1:length(q0q1q2_Pos_mat_NewAdjust)- 0
        
        %========================== Animation ========= ===================
        ReconbotANI(q0q1q2_Pos_mat_NewAdjust(i,:));
%         set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%         set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
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