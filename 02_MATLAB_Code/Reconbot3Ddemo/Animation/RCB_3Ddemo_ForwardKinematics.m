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
        BaseLow_CenterPointBearing = [250, 250, 83.5];
        %
        BaseUP_CenterPointBearing = [160, 110, 60.44];
        BaseUP_CenterPointA1C1 = [45, 110, 0];
        BaseUP_CenterPointA2C2 = [275, 110, 0];
        %
        BaseJointA1C1A2C2_CenterPointMotor = [34.5, 48.1, 45.5];
        BaseJointA1C1A2C2_CenterPointSidePlate = [58.5, 48.1, 22];
        %
        LowUpLinkA1C1_CenterPointMotor = [20.10, 164.75, 45.5];
        LowUpLinkA1C1_CenterPointBearing = [20.10, 17.5, 76.07];
        %
        LowLinkA2C2_CenterPointMotor = [20.10, 163.13, 45.5];
        LowLinkA2C2_CenterPointBearing = [20.10, 15.88, 71.07];
        %
        UpLinkA2C2_CenterPointMotor = [20.10, 164.75, 50.15];
        UpLinkA2C2_CenterPointBearing = [20.10, 17.5, 14.57];
        %
        UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2 = [54.65, 17.5, 15];
        UPjointA1C1A2C2_CenterPointConnectWithMP = [17.5, 17.5, 69.48];
        %
        MovingPlatform_CenterPointUPside = [160, 20, 40];
        MovingPlatform_CenterPointA1C1 = [45, 20, 50];
        MovingPlatform_CenterPointA2C2 = [275, 20, 50];
        % %--------------------------------------------------------------------
        %%
        %------------------------ Base Platform --------------------
        % Forward Kinematics - BaseUp
        Trans_BaseLow_CenterPointBearing = [[eye(3,3);0,0,0], [BaseLow_CenterPointBearing, 1]'];
        Trans_BaseUP_CenterPoint = [[eye(3,3);0,0,0],[- BaseUP_CenterPointBearing, 1]'];        
        T_BaseUP_Trans2OrigPoint_Rotq0 = Trans_BaseLow_CenterPointBearing *  tmat(0, 0, 0, q0) * rotm2tform(roty(180)) *...
            rotm2tform(rotz(0)) *  Trans_BaseUP_CenterPoint;
        
                
        %------------------------ Chain A1C1 ------------------------
        % Forward Kinematics - BaseJointA1C1
        Trans_BaseUP_CenterPointA1C1 = [[eye(3,3);0,0,0], [BaseUP_CenterPointA1C1, 1]'];
        Trans_BaseJointA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- BaseJointA1C1A2C2_CenterPointMotor, 1]'];  
        T_1_01 = Trans_BaseUP_CenterPointA1C1 * rotm2tform(rotz(- 90)) * rotm2tform(rotz( - q11)) * Trans_BaseJointA1C1_CenterPointMotor;
        
        % Forward Kinematics - LowLinkA1C1
        Trans_BaseJointA1C1_CenterPointSidePlate = [[eye(3,3);0,0,0], [BaseJointA1C1A2C2_CenterPointSidePlate, 1]'];
        Trans_LowLinkA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- LowUpLinkA1C1_CenterPointMotor, 1]'];        
        T_1_12 = Trans_BaseJointA1C1_CenterPointSidePlate * rotm2tform(rotx(q12)) * rotm2tform(roty(-90)) * Trans_LowLinkA1C1_CenterPointMotor;
        
        % Forward Kinematics - UpLinkA1C1
        Trans_UpLinkA1C1_CenterPointBearing_Postive = [[eye(3,3);0,0,0], [- LowUpLinkA1C1_CenterPointBearing, 1]'];
        Trans_UpLinkA1C1_CenterPointMotor_Negative = [[eye(3,3);0,0,0],[LowUpLinkA1C1_CenterPointBearing, 1]'];        
        T_1_23 = Trans_UpLinkA1C1_CenterPointMotor_Negative * rotm2tform(rotz(- q13)) *...
             rotm2tform(rotz(180) * roty(180)) * Trans_UpLinkA1C1_CenterPointBearing_Postive;
        
        % Forward Kinematics - UPjointA1C1
        Trans_UpLinkA1C1_CenterPointMotor = [[eye(3,3);0,0,0], [LowUpLinkA1C1_CenterPointMotor, 1]'];
        Trans_UPjointA1C1_CenterPointConnectWithA1C1 = [[eye(3,3);0,0,0],[- UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2, 1]'];        
        T_1_34 = Trans_UpLinkA1C1_CenterPointMotor * rotm2tform(rotz( q14)) * rotm2tform(rotz(90) * roty(90)) * Trans_UPjointA1C1_CenterPointConnectWithA1C1;
        
        %---------------------- Moving Platform ------------------------
        % Forward Kinematics - Moving Platform
        Trans_UPjointA1C1A2C2_CenterPointConnectWithMP = [[eye(3,3);0,0,0], [UPjointA1C1A2C2_CenterPointConnectWithMP, 1]'];
        Trans_MovingPlatform_CenterPointA1C1 = [[eye(3,3);0,0,0],[- MovingPlatform_CenterPointA1C1, 1]'];        
        T_1_45 = Trans_UPjointA1C1A2C2_CenterPointConnectWithMP * rotm2tform(rotz(- q15)) * rotm2tform(rotz(90) * rotx(180)) * Trans_MovingPlatform_CenterPointA1C1;
        
        %------------------------ Chain A2C2 ------------------------
        % Forward Kinematics - BaseJointA2C2
        Trans_BaseUP_CenterPointA2C2 = [[eye(3,3);0,0,0], [BaseUP_CenterPointA2C2, 1]'];
        Trans_BaseJointA1C1_CenterPointMotor = [[eye(3,3);0,0,0],[- BaseJointA1C1A2C2_CenterPointMotor, 1]'];    
        T_2_01 = Trans_BaseUP_CenterPointA2C2 * rotm2tform(rotz(90)) * rotm2tform(rotz( - q21)) * Trans_BaseJointA1C1_CenterPointMotor;
        
        % Forward Kinematics - LowLinkA2C2
        Trans_BaseJointA2C2_CenterPointSidePlate = [[eye(3,3);0,0,0], [BaseJointA1C1A2C2_CenterPointSidePlate, 1]'];
        Trans_LowLinkA2C2_CenterPointMotor = [[eye(3,3);0,0,0],[- LowLinkA2C2_CenterPointMotor, 1]']; 
        T_2_12 = Trans_BaseJointA2C2_CenterPointSidePlate * rotm2tform(rotx(q22)) * rotm2tform(roty(-90)) * Trans_LowLinkA2C2_CenterPointMotor;
        
        % Forward Kinematics - UpLinkA2C2
        Trans_UpLinkA2C2_CenterPointBearing_Postive = [[eye(3,3);0,0,0], [LowLinkA2C2_CenterPointBearing, 1]'];
        Trans_UpLinkA2C2_CenterPointMotor_Negative = [[eye(3,3);0,0,0],[- UpLinkA2C2_CenterPointMotor, 1]'];        
        T_2_23 = Trans_UpLinkA2C2_CenterPointBearing_Postive * rotm2tform(rotz( - q23)) * rotm2tform(roty(180)) * Trans_UpLinkA2C2_CenterPointMotor_Negative;
        
        % Forward Kinematics - UPjointA2C2
        Trans_UpLinkA2C2_CenterPointMotor = [[eye(3,3);0,0,0], [UpLinkA2C2_CenterPointBearing, 1]'];
        Trans_UPjointA2C2_CenterPointConnectWithA2C2 = [[eye(3,3);0,0,0],[- UPjointA1C1A2C2_CenterPointConnectWithA1C1A2C2, 1]'];        
        T_2_34 = Trans_UpLinkA2C2_CenterPointMotor * rotm2tform(rotz(q24)) * rotm2tform(rotz(-90) * roty(90)) * Trans_UPjointA2C2_CenterPointConnectWithA2C2;
        %--------------------------------------------------------------   
             
        %---------------------- Transformation Matrix --------------------
        % BaseUp
        T_01 = T_BaseUP_Trans2OrigPoint_Rotq0;
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