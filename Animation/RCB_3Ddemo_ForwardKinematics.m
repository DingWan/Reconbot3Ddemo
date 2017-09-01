function [T_01,T_1_02,T_1_03,T_1_04,T_1_05,T_1_06,T_2_02,T_2_03,T_2_04,T_2_05] = RCB_3Ddemo_ForwardKinematics(q0q1q2)
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI

%         BaseLow_data = getappdata(0,'Link_BaseLow_data');
%         BaseUP_data = getappdata(0,'Link_BaseUP_data');
%         BaseJointA1C1_data = getappdata(0,'Link_BaseJointA1C1_data');
%         LowLinkA1C1_data = getappdata(0,'Link_LowLinkA1C1_data');
%         UpLinkA1C1_data = getappdata(0,'Link_UpLinkA1C1_data');
%         UPjointA1C1_data = getappdata(0,'Link_UPjointA1C1_data');
%         BaseJointA2C2_data = getappdata(0,'Link_BaseJointA2C2_data');
%         LowLinkA2C2_data = getappdata(0,'Link_LowLinkA2C2_data');
%         UpLinkA2C2_data = getappdata(0,'Link_UpLinkA2C2_data');
%         UPjointA2C2_data = getappdata(0,'Link_UPjointA2C2_data');
%         MovingPlatform_data = getappdata(0,'Link_MovingPlatform_data');        
%         %      

        %The 'home' position, for init.
         q0 = q0q1q2(1);
        q11 = q0q1q2(2);
        q12 = q0q1q2(3);
        q13 = q0q1q2(4);
        q14 = q0q1q2(5);
        q15 = q0q1q2(6);
        q21 = q0q1q2(7);
        q22 = q0q1q2(8);
        q23 = q0q1q2(9);
        q24 = q0q1q2(10);
        q25 = q0q1q2(11);
        
        % %----------------------------------------------------------------
        % Link connection point displacement
        BaseLow_CenterPointBearing = [250, 250, 88.97];
        %
        BaseUP_CenterPointBearing = [170, 160, 5.47];
        BaseUP_CenterPointA1C1 = [170, 50, 58.88];
        BaseUP_CenterPointA2C2 = [170, 270, 58.88];
        %
        BaseJointA1C1A2C2_CenterPointMotor = [24, 48.10, 4.5];
        BaseJointA1C1A2C2_CenterPointSidePlate = [0, 48.10, 28];
        
        LowUpLinkA1C1A2C2_CenterPointMotor = [20.10, 163.13, 30.57];
        LowUpLinkA1C1A2C2_CenterPointBearing = [20.10, 15.88, 0];
        %
        UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2 = [16.76, 15, 0];
        UPjointA1C1A2C2_CenterPointConnectWithMP = [16.76, 64.29, 37.15];
        %
        MovingPlatform_CenterPointUPside = [170, 160, 54.26];
        MovingPlatform_CenterPointA1C1 = [170, 50, 0];
        MovingPlatform_CenterPointA2C2 = [170, 270, 0];
        % %--------------------------------------------------------------------
        %%
        %------------------------ Base Platform --------------------
        % Forward Kinematics - BaseUp
        Trans_BaseLow_CenterPointBearing = [[eye(3,3);0,0,0], [BaseLow_CenterPointBearing, 1]'];
        Trans_BaseUP_CenterPoint = [[eye(3,3);0,0,0],[- BaseUP_CenterPointBearing, 1]'];        
        T_BaseUP_Trans2OrigPoint_Rotq0 =  tmat(0, 0, 0, q0) * Trans_BaseUP_CenterPoint;
                
        %------------------------ Chain A1C1 ------------------------
        % Forward Kinematics - BaseJointA1C1
        Trans_BaseUP_CenterPointA1C1 = [[eye(3,3);0,0,0], [BaseUP_CenterPointA1C1, 1]'];
        Trans_BaseJointA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- BaseJointA1C1A2C2_CenterPointMotor, 1]'];        
        T_BaseJointA1C1_Trans2OrigPoint_Rotq11 =  tmat(0, 0, 0, q11) * Trans_BaseJointA1C1_CenterPointMotor;
        T_1_01 = Trans_BaseUP_CenterPointA1C1 * T_BaseJointA1C1_Trans2OrigPoint_Rotq11;
        
        % Forward Kinematics - LowLinkA1C1
        Trans_BaseJointA1C1_CenterPointSidePlate = [[eye(3,3);0,0,0], [BaseJointA1C1A2C2_CenterPointSidePlate, 1]'];
        Trans_LowLinkA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- LowUpLinkA1C1A2C2_CenterPointMotor, 1]'];        
        T_LowLinkA1C1_Trans2OrigPoint_Rotq11 =  rotm2tform(roty(-90)) * tmat(0, 0, 0, q12) * Trans_LowLinkA1C1_CenterPointMotor;
        T_1_12 = Trans_BaseJointA1C1_CenterPointSidePlate * T_LowLinkA1C1_Trans2OrigPoint_Rotq11;
        
        % Forward Kinematics - UpLinkA1C1
        Trans_UpLinkA1C1_CenterPointBearing_Postive = [[eye(3,3);0,0,0], [LowUpLinkA1C1A2C2_CenterPointBearing, 1]'];
        Trans_UpLinkA1C1_CenterPointMotor_Negative = [[eye(3,3);0,0,0],[- LowUpLinkA1C1A2C2_CenterPointBearing, 1]'];        
        T_1_23 = Trans_UpLinkA1C1_CenterPointBearing_Postive * tmat(0, 0, 0, q13) *...
             rotm2tform(rotz(180) * roty(180)) * Trans_UpLinkA1C1_CenterPointMotor_Negative;
        
        % Forward Kinematics - UPjointA1C1
        Trans_UpLinkA1C1_CenterPointMotor = [[eye(3,3);0,0,0], [LowUpLinkA1C1A2C2_CenterPointMotor, 1]'];
        Trans_UPjointA1C1_CenterPointConnectWithA1C1 = [[eye(3,3);0,0,0],[- UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2, 1]'];        
        T_1_34 = Trans_UpLinkA1C1_CenterPointMotor * tmat(0, 0, 0, q14) * rotm2tform(roty(180)) * Trans_UPjointA1C1_CenterPointConnectWithA1C1;
        
        %---------------------- Moving Platform ------------------------
        % Forward Kinematics - Moving Platform
        Trans_UPjointA1C1A2C2_CenterPointConnectWithMP = [[eye(3,3);0,0,0], [UPjointA1C1A2C2_CenterPointConnectWithMP, 1]'];
        Trans_MovingPlatform_CenterPointA1C1 = [[eye(3,3);0,0,0],[- MovingPlatform_CenterPointA1C1, 1]'];        
        T_1_45 = Trans_UPjointA1C1A2C2_CenterPointConnectWithMP * rotm2tform(roty(90) * rotx(-90)) * tmat(0, 0, 0, q15) * Trans_MovingPlatform_CenterPointA1C1;
        
        %------------------------ Chain A2C2 ------------------------
        % Forward Kinematics - BaseJointA2C2
        Trans_BaseUP_CenterPointA2C2 = [[eye(3,3);0,0,0], [BaseUP_CenterPointA2C2, 1]'];
        Trans_BaseJointA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- BaseJointA1C1A2C2_CenterPointMotor, 1]'];        
        T_BaseJointA1C1_Trans2OrigPoint_Rotq11 = rotm2tform(rotz(180)) * tmat(0, 0, 0, q21) * Trans_BaseJointA1C1_CenterPointMotor;
        T_2_01 = Trans_BaseUP_CenterPointA2C2 * T_BaseJointA1C1_Trans2OrigPoint_Rotq11;
        
        % Forward Kinematics - LowLinkA2C2
        Trans_BaseJointA2C2_CenterPointSidePlate = [[eye(3,3);0,0,0], [BaseJointA1C1A2C2_CenterPointSidePlate, 1]'];
        Trans_LowLinkA2C2_CenterPointMotor = [[eye(3,3);0,0,0],[- LowUpLinkA1C1A2C2_CenterPointMotor, 1]'];        
        T_LowLinkA2C2_Trans2OrigPoint_Rotq11 =  rotm2tform(roty(-90)) * tmat(0, 0, 0, q22) * Trans_LowLinkA2C2_CenterPointMotor;
        T_2_12 = Trans_BaseJointA2C2_CenterPointSidePlate * T_LowLinkA2C2_Trans2OrigPoint_Rotq11;
        
        % Forward Kinematics - UpLinkA2C2
        Trans_UpLinkA2C2_CenterPointBearing_Postive = [[eye(3,3);0,0,0], [LowUpLinkA1C1A2C2_CenterPointBearing, 1]'];
        Trans_UpLinkA2C2_CenterPointMotor_Negative = [[eye(3,3);0,0,0],[- LowUpLinkA1C1A2C2_CenterPointBearing, 1]'];        
        T_2_23 = Trans_UpLinkA2C2_CenterPointBearing_Postive * tmat(0, 0, 0, q23) *...
            rotm2tform(rotz(180)) * rotm2tform(roty(180)) * Trans_UpLinkA2C2_CenterPointMotor_Negative;
        
        % Forward Kinematics - UPjointA2C2
        Trans_UpLinkA2C2_CenterPointMotor = [[eye(3,3);0,0,0], [LowUpLinkA1C1A2C2_CenterPointMotor, 1]'];
        Trans_UPjointA2C2_CenterPointConnectWithA2C2 = [[eye(3,3);0,0,0],[- UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2, 1]'];        
        T_2_34 = Trans_UpLinkA2C2_CenterPointMotor * tmat(0, 0, 0, q24) * rotm2tform(roty(180)) * Trans_UPjointA2C2_CenterPointConnectWithA2C2;
        %--------------------------------------------------------------   
             
        %---------------------- Transformation Matrix --------------------
        % BaseUp
        T_01 = Trans_BaseLow_CenterPointBearing * T_BaseUP_Trans2OrigPoint_Rotq0;
        % Each link frame A1C1 to base frame transformation
        T_1_02 = T_01 * T_1_01;
        T_1_03 = T_1_02 * T_1_12;
        T_1_04 = T_1_03 * T_1_23;
        T_1_05 = T_1_04 * T_1_34;
        T_1_06 = T_1_05 * T_1_45;         
        % Each link fram A2C2 to base frame transformation
        T_2_02 = T_01 * T_2_01;
        T_2_03 = T_2_02 * T_2_12;
        T_2_04 = T_2_03 * T_2_23;
        T_2_05 = T_2_04 * T_2_34;
        %T_2_06 = T_2_05 * T_2_45;
        
%         % Actual vertex data of robot links
%         Link_BaseLow        =           BaseLow_data.V1;
%         Link_BaseUP         = (T_01   * BaseUP_data.V2')';
%         Link_BaseJointA1C1  = (T_1_02 * BaseJointA1C1_data.V3')';
%         Link_LowLinkA1C1    = (T_1_03 * LowLinkA1C1_data.V4')';
%         Link_UpLinkA1C1     = (T_1_04 * UpLinkA1C1_data.V5')';
%         Link_UPjointA1C1    = (T_1_05 * UPjointA1C1_data.V6')';
%         
%         Link_MovingPlatform = (T_1_06 * MovingPlatform_data.V11')';
%         
%         Link_BaseJointA2C2  = (T_2_02 * BaseJointA2C2_data.V7')';
%         Link_LowLinkA2C2    = (T_2_03 * LowLinkA2C2_data.V8')';
%         Link_UpLinkA2C2     = (T_2_04 * UpLinkA2C2_data.V9')';
%         Link_UPjointA2C2    = (T_2_05 * UPjointA2C2_data.V10')';
%         
%         % points are no fun to watch, make it look 3d.
%         L1 = patch('faces', BaseLow_data.F1, 'vertices' ,Link_BaseLow(:,1:3));
%         L2 = patch('faces', BaseUP_data.F2, 'vertices' ,Link_BaseUP(:,1:3));
%         L3 = patch('faces', BaseJointA1C1_data.F3, 'vertices' ,Link_BaseJointA1C1(:,1:3));
%         L4 = patch('faces', LowLinkA1C1_data.F4, 'vertices' ,Link_LowLinkA1C1(:,1:3));
%         L5 = patch('faces', UpLinkA1C1_data.F5, 'vertices' ,Link_UpLinkA1C1(:,1:3));
%         L6 = patch('faces', UPjointA1C1_data.F6, 'vertices' ,Link_UPjointA1C1(:,1:3));
%         
%         L11 = patch('faces', MovingPlatform_data.F11, 'vertices' ,Link_MovingPlatform(:,1:3));
%         
%         L7 = patch('faces', BaseJointA2C2_data.F7, 'vertices' ,Link_BaseJointA2C2(:,1:3));
%         L8 = patch('faces', LowLinkA2C2_data.F8, 'vertices' ,Link_LowLinkA2C2(:,1:3));
%         L9 = patch('faces', UpLinkA2C2_data.F9, 'vertices' ,Link_UpLinkA2C2(:,1:3));
%        L10 = patch('faces', UPjointA2C2_data.F10, 'vertices' ,Link_UPjointA2C2(:,1:3));
%        
%         Tr = plot3(0,0,0,'b.'); % holder for trail paths
%         %
%         setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11])
%         %
%         setappdata(0,'xtrail',0); % used for trail tracking.
%         setappdata(0,'ytrail',0); % used for trail tracking.
%         setappdata(0,'ztrail',0); % used for trail tracking.
%         %
%         set(L1, 'facec', [105 105 105]/255);%105 105 105
%         set(L1, 'EdgeColor','none');
%         set(L2, 'facec', [250 235 215]/255);
%         set(L2, 'EdgeColor','none');
%         set(L3, 'facec', [105 105 105]/255);
%         set(L3, 'EdgeColor','none');
%         set(L4, 'facec', [255 182 193]/255);
%         set(L4, 'EdgeColor','none');
%         set(L5, 'facec', [135 206 250]/255);
%         set(L5, 'EdgeColor','none');
%         set(L6, 'facec', [105 105 105]/255);
%         set(L6, 'EdgeColor','none');
%         
%         set(L11, 'facec', [250 235 215]/255);
%         set(L11, 'EdgeColor','none');
%                 
%         set(L7, 'facec', [105 105 105]/255);
%         set(L7, 'EdgeColor','none');
%         set(L8, 'facec', [255 182 193]/255);
%         set(L8, 'EdgeColor','none');
%         set(L9, 'facec', [135 206 250]/255);
%         set(L9, 'EdgeColor','none');
%         set(L10, 'facec', [105 105 105]/255);
%         set(L10, 'EdgeColor','none');
%         %
        %drawnow
end
    
    function T = tmat(alpha, a, d, theta)
        % tmat(alpha, a, d, theta) (T-Matrix used in Robotics)
        % The homogeneous transformation called the "T-MATRIX"
        % as used in the Kinematic Equations for robotic type
        % systems (or equivalent).
        %
        % This is equation 3.6 in Craig's "Introduction to Robotics."
        % alpha, a, d, theta are the Denavit-Hartenberg parameters.
        %
        % (NOTE: ALL ANGLES MUST BE IN DEGREES.)
        %
        alpha = alpha*pi/180;    %Note: alpha is in radians.
        theta = theta*pi/180;    %Note: theta is in radians.
        c = cos(theta);
        s = sin(theta);
        ca = cos(alpha);
        sa = sin(alpha);
        T = [c -s 0 a; s*ca c*ca -sa -sa*d; s*sa c*sa ca ca*d; 0 0 0 1];
    end