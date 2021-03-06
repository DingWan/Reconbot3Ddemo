function [LinkA1C1_ColChe, LinkA2C2_ColChe, BaseUpPlatform_ColChe] = CollisionPointsFK(q0q1q2)
%[T_01,T_1_02,T_1_03,T_1_04,T_1_05,T_1_06,T_2_02,T_2_03,T_2_04,T_2_05] = CollisionPointsFK(q0q1q2)
        % Use forward kinematics to place the robot in a specified
        % configuration.
        % Figure setup data, create a new figure for the GUI
        
        %q0q1q2 = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0] * 180/pi;
        % %----------------------------------------------------------------
                % Link connection point for collision check
                %Base Low
                BaseLow_Xaxis = [0, 171, 80;   0, 211, 80;   0, 289, 80;   0, 329, 80];
                BaseLow_Yaxis = [500, 171, 80;   500, 211, 80;   500, 289, 80;   500, 329, 80];
                BaseLow_CollisionCheck = [BaseLow_Xaxis, ones(4, 1); BaseLow_Yaxis, ones(4, 1)];
                %Base Up
                BaseUP_Xaxis = [0, 140, 43.97;   0, 180, 43.97;   150, 320, 43.97;   190, 320, 43.97];
                BaseUP_Yaxis = [340, 140, 43.97;   340, 180, 43.97;   150, 0, 43.97;   190, 0, 43.97];
                BaseUP_CollisionCheck = [BaseUP_Xaxis, ones(4, 1); BaseUP_Yaxis, ones(4, 1)];
                %LinkA1C1/A2C2
                    % Method I: Large potection area : Cuboid ������
                    %LinkA1C1A2C2_Motor = [0, 176.15, 0;   40.20, 176.15, 0;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
                    %LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 76.07;   0, 0, 76.07;];
                    % Method II: Small potection area : Oblique Prism б����
                    LinkA1C1A2C2_Motor = [0, 176.15, 30.57;   40.20, 176.15, 30.57;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
                    LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 35.02;   0, 0, 35.02;];
                    LinkA1C1A2C2_CollisionCheck = [LinkA1C1A2C2_Motor, ones(4, 1); LinkA1C1A2C2_Bearing, ones(4, 1)];
                %Moving Platform
                MP_Xaxis = [0, 140, 14.26;   0, 150, 14.26;   150, 320, 14.26;   190, 320, 14.26];
                MP_Yaxis = [340, 140, 14.26;   340, 180, 14.26;   150, 0, 14.26;   190, 0, 14.26];
                MP_CollisionCheck = [MP_Xaxis, ones(4, 1); MP_Yaxis, ones(4, 1)];
        % %--------------------------------------------------------------------
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
        
        % Actual vertex data of robot links
        BaseLow_ColChe        =           BaseLow_CollisionCheck;
        BaseUP_ColChe         = (T_01   * BaseUP_CollisionCheck')';
        
        LowLinkA1C1_ColChe    = (T_1_03 * LinkA1C1A2C2_CollisionCheck')';
        UpLinkA1C1_ColChe     = (T_1_04 * LinkA1C1A2C2_CollisionCheck')';
        
        MP_ColChe             = (T_1_06 * MP_CollisionCheck')';
        
        LowLinkA2C2_ColChe    = (T_2_03 * LinkA1C1A2C2_CollisionCheck')';
        UpLinkA2C2_ColChe     = (T_2_04 * LinkA1C1A2C2_CollisionCheck')';
        
        %[BaseLow_ColChe, BaseUP_ColChe, LowLinkA1C1_ColChe, UpLinkA1C1_ColChe, LowLinkA2C2_ColChe, UpLinkA2C2_ColChe, MP_ColChe]
        
        LinkA1C1_ColChe = [LowLinkA1C1_ColChe(:,1:3); UpLinkA1C1_ColChe(:,1:3)];
        LinkA2C2_ColChe = [LowLinkA2C2_ColChe(:,1:3); UpLinkA2C2_ColChe(:,1:3)];
        BaseUpPlatform_ColChe = [BaseLow_ColChe(:,1:3); BaseUP_ColChe; MP_ColChe(:,1:3)];
        
        %--------------------------------------------------------------
        %LowLinkA1C1_ColChe
        plot3(LowLinkA1C1_ColChe(:,1), LowLinkA1C1_ColChe(:,2), LowLinkA1C1_ColChe(:,3),'b-o'); hold on        
        %UpLinkA1C1_ColChe 
        plot3(UpLinkA1C1_ColChe(:,1), UpLinkA1C1_ColChe(:,2), UpLinkA1C1_ColChe(:,3),'g-o'); hold on        
        %LowLinkA2C2_ColChe
        plot3(LowLinkA2C2_ColChe(:,1), LowLinkA2C2_ColChe(:,2), LowLinkA2C2_ColChe(:,3),'b-o'); hold on        
        %UpLinkA2C2_ColChe
        plot3(UpLinkA2C2_ColChe(:,1), UpLinkA2C2_ColChe(:,2), UpLinkA2C2_ColChe(:,3),'g-o'); hold on        
        %BaseLow_ColChe
        plot3(BaseLow_ColChe(:,1), BaseLow_ColChe(:,2),BaseLow_ColChe(:,3),'k-o'); hold on                
        %BaseUP_ColChe
        plot3(BaseUP_ColChe(:,1), BaseUP_ColChe(:,2),BaseUP_ColChe(:,3),'k-o'); hold on        
        %MMP_ColChe
        plot3(MP_ColChe(:,1), MP_ColChe(:,2), MP_ColChe(:,3),'k-o'); hold on
        %--------------------------------------------------------------

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