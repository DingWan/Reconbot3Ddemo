function [CollisionOccur_DistanceA1C1, CP_A1C1, CollisionOccur_DistanceA2C2, CP_A2C2] = CollisionCheck(q0q1q2)
        % Collision check for the set of collision points on each link; 
        % Mainly calculate the distance between two segements;
        
%% Collision Points at its own coordinate frame
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
            % Method I: Large potection area : Cuboid ≥§∑ΩÃÂ
            %LinkA1C1A2C2_Motor = [0, 176.15, 0;   40.20, 176.15, 0;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
            %LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 76.07;   0, 0, 76.07;];
            % Method II: Small potection area : Oblique Prism –±¿‚÷˘
            LinkA1C1A2C2_Motor = [0, 176.15, 30.57;   40.20, 176.15, 30.57;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
            LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 35.02;   0, 0, 35.02;];
            LinkA1C1A2C2_CollisionCheck = [LinkA1C1A2C2_Motor, ones(4, 1); LinkA1C1A2C2_Bearing, ones(4, 1)];
            
        %Moving Platform
        MP_Xaxis = [0, 140, 14.26;   0, 150, 14.26;   150, 320, 14.26;   190, 320, 14.26];
        MP_Yaxis = [340, 140, 14.26;   340, 180, 14.26;   150, 0, 14.26;   190, 0, 14.26];
        MP_CollisionCheck = [MP_Xaxis, ones(4, 1); MP_Yaxis, ones(4, 1)];
                
%% FK for all collision points at world coordinate frame     
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
        % Forward Kinematics - BaseLow
        %Displacement = [250,250,167.4400];
        %Trans_BaseLow_2RERorigin = [[eye(3,3);0,0,0], [- Displacement, 1]'];
        %T_BaseLow_TransTo2RERorigin = rotm2tform(rotz(0)) * Trans_BaseLow_2RERorigin; 
        
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
        %T_00 = T_BaseLow_TransTo2RERorigin;
        % BaseUp
        T_01 = T_BaseUP_Trans2OrigPoint_Rotq0;% * T_00;
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
        
        %---------------------- Collision Points --------------------
        % the collision points after transformation
        BaseLow_ColChe        =           BaseLow_CollisionCheck;
        BaseUP_ColChe         = (T_01   * BaseUP_CollisionCheck')';
        
        LowLinkA1C1_ColChe    = (T_1_03 * LinkA1C1A2C2_CollisionCheck')';
        UpLinkA1C1_ColChe     = (T_1_04 * LinkA1C1A2C2_CollisionCheck')';
        
        MP_ColChe             = (T_1_06 * MP_CollisionCheck')';
        
        LowLinkA2C2_ColChe    = (T_2_03 * LinkA1C1A2C2_CollisionCheck')';
        UpLinkA2C2_ColChe     = (T_2_04 * LinkA1C1A2C2_CollisionCheck')';

%%        
        %----------------------- Combine collision points ---------------------------
        % Combine collision points A1B1C1(LinkA1C1_ColChe),
        % A2B2C2(LinkA1C1_ColChe), BaseLow, BaseUp, and up platform (BaseUpPlatform_ColChe)
        LinkA1C1_ColChe = [LowLinkA1C1_ColChe(1:4,1:3), LowLinkA1C1_ColChe(5:8,1:3); UpLinkA1C1_ColChe(1:4,1:3), UpLinkA1C1_ColChe(5:8,1:3)];
        LinkA2C2_ColChe = [LowLinkA2C2_ColChe(1:4,1:3), LowLinkA2C2_ColChe(5:8,1:3); UpLinkA2C2_ColChe(1:4,1:3), UpLinkA2C2_ColChe(5:8,1:3)];
        BaseUpPlatform_ColChe = [BaseLow_ColChe(1:4,1:3), BaseLow_ColChe(5:8,1:3); BaseUP_ColChe(1:4,1:3), BaseUP_ColChe(5:8,1:3); MP_ColChe(1:4,1:3),MP_ColChe(5:8,1:3)];
        
%         %------------------------- Plot collision points -----------------------------
%         %LowLinkA1C1_ColChe
%         plot3(LowLinkA1C1_ColChe(:,1), LowLinkA1C1_ColChe(:,2), LowLinkA1C1_ColChe(:,3),'b-'); hold on        
%         %UpLinkA1C1_ColChe 
%         plot3(UpLinkA1C1_ColChe(:,1), UpLinkA1C1_ColChe(:,2), UpLinkA1C1_ColChe(:,3),'g-'); hold on        
%         %LowLinkA2C2_ColChe
%         plot3(LowLinkA2C2_ColChe(:,1), LowLinkA2C2_ColChe(:,2), LowLinkA2C2_ColChe(:,3),'b-'); hold on        
%         %UpLinkA2C2_ColChe
%         plot3(UpLinkA2C2_ColChe(:,1), UpLinkA2C2_ColChe(:,2), UpLinkA2C2_ColChe(:,3),'g-'); hold on        
%         %BaseLow_ColChe
%         plot3(BaseLow_ColChe(:,1), BaseLow_ColChe(:,2),BaseLow_ColChe(:,3),'k-'); hold on                
%         %BaseUP_ColChe
%         plot3(BaseUP_ColChe(:,1), BaseUP_ColChe(:,2),BaseUP_ColChe(:,3),'k-'); hold on        
%         %MMP_ColChe
%         plot3(MP_ColChe(:,1), MP_ColChe(:,2), MP_ColChe(:,3),'k-'); hold on
%         %--------------------------------------------------------------

        
%% Obtain the shortest distance,  and the possible collision two links and points on each collision link
        % LinkA1C1_ColChe with BaseUpPlatform_ColChe  (Shortest disance)     
        for i = 1 : length(LinkA1C1_ColChe(:,1))
            for j = 1 : length(BaseUpPlatform_ColChe(:,1))
                P1 = LinkA1C1_ColChe(i,1:3);           P2 = LinkA1C1_ColChe(i,4:6);
                P3 = BaseUpPlatform_ColChe(j,1:3);     P4 = BaseUpPlatform_ColChe(j,4:6);
                [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
                LinkA1C1_BaseUpPlatform_distance(i,j) = distance;
            end
        end

        % LinkA2C2_ColChe with BaseUpPlatform_ColChe  (Shortest disance)
        for i = 1 : length(LinkA2C2_ColChe(:,1))
            for j = 1 : length(BaseUpPlatform_ColChe(:,1))
                P1 = LinkA2C2_ColChe(i,1:3);           P2 = LinkA2C2_ColChe(i,4:6);
                P3 = BaseUpPlatform_ColChe(j,1:3);     P4 = BaseUpPlatform_ColChe(j,4:6);
                [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
                LinkA2C2_BaseUpPlatform_distance(i,j) = distance;
            end
        end

        % LinkA1C1_ColChe with LinkA2C2_ColChe  (Shortest disance)
        for i = 1 : length(LinkA1C1_ColChe(:,1))
            for j = 1 : length(LinkA2C2_ColChe(:,1))
                P1 = LinkA1C1_ColChe(i,1:3);     P2 = LinkA1C1_ColChe(i,4:6);
                P3 = LinkA2C2_ColChe(j,1:3);     P4 = LinkA2C2_ColChe(j,4:6);
                [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
                LinkA1C1_LinkA2C2_ColChe_distance(i,j) = distance;
            end
        end
        
        %---------------------------Collision Check for AiCi----------------------
        % Combine all the distance value of A1C1 in to Matrix "Total_distance"
        Total_distance_A1C1 = [LinkA1C1_BaseUpPlatform_distance, LinkA1C1_LinkA2C2_ColChe_distance];
        % Find the row and column of the shortest distance of the A1C1 of the Matrix "Total_distance"
        [rows_A1C1,cols_A1C1] = find(Total_distance_A1C1 == min(min(Total_distance_A1C1)));
        % Assign the value to the two lines (4 points, P1-P4)
        if cols_A1C1(1) < 13
            P1 = LinkA1C1_ColChe(rows_A1C1(1),1:3);             P2 = LinkA1C1_ColChe(rows_A1C1(1),4:6);
            P3 = BaseUpPlatform_ColChe(cols_A1C1(1),1:3);       P4 = BaseUpPlatform_ColChe(cols_A1C1(1),4:6);
        elseif cols_A1C1(1) > 12 
            P1 = LinkA1C1_ColChe(rows_A1C1(1),1:3);             P2 = LinkA1C1_ColChe(rows_A1C1(1),4:6);
            P3 = LinkA2C2_ColChe(cols_A1C1(1)-12,1:3);          P4 = LinkA2C2_ColChe(cols_A1C1(1)-12,4:6);
        end
        % Assign the value of the shortest distance and two Closed Points.
        [distance_A1C1, ClosePoint_A1C1] = DistBetween2Segment(P1, P2, P3, P4);        
        % Plot the Collision points
%         P12 = [P1;P2];  P34 = [P3;P4];
        CP_A1C1 = [ClosePoint_A1C1{2};ClosePoint_A1C1{3}];
%         plot3(P12(:,1),P12(:,2),P12(:,3),'b-');
%         plot3(P34(:,1),P34(:,2),P34(:,3),'b-');
%         Handle_CP = plot3(CP(:,1),CP(:,2),CP(:,3),'r-');
        if distance_A1C1 < 5
            CollisionOccur = 1;
        else
            CollisionOccur = 0;
        end
        CollisionOccur_DistanceA1C1 = [CollisionOccur, distance_A1C1];
        
        %------------------------Collision Check for A2C2----------------------
        % Combine all the distance value of A2C2 in to Matrix "Total_distance"
        Total_distance_A2C2 = [LinkA2C2_BaseUpPlatform_distance, LinkA1C1_LinkA2C2_ColChe_distance];
        % Find the row and column of the shortest distance of A2C2 of the Matrix "Total_distance"
        [rows_A2C2,cols_A2C2] = find(Total_distance_A2C2 == min(min(Total_distance_A2C2)));
        % Assign the value to the two lines (4 points, P1-P4)
        if cols_A2C2(1) < 13
            P1 = LinkA2C2_ColChe(rows_A2C2(1),1:3);           P2 = LinkA2C2_ColChe(rows_A2C2(1),4:6);
            P3 = BaseUpPlatform_ColChe(cols_A2C2(1),1:3);     P4 = BaseUpPlatform_ColChe(cols_A2C2(1),4:6);
        elseif cols_A2C2(1) > 12
            P1 = LinkA1C1_ColChe(rows_A1C1(1),1:3);             P2 = LinkA1C1_ColChe(rows_A1C1(1),4:6);
            P3 = LinkA2C2_ColChe(cols_A1C1(1)-12,1:3);          P4 = LinkA2C2_ColChe(cols_A1C1(1)-12,4:6);
        end
        % Assign the value of the shortest distance and two Closed Points.
        [distance_A2C2, ClosePoint_A2C2] = DistBetween2Segment(P1, P2, P3, P4);
        
        % Plot the Collision points
        CP_A2C2 = [ClosePoint_A2C2{2};ClosePoint_A2C2{3}];
        % Collision Check
        if distance_A2C2 < 5
            CollisionOccur = 1;
        else
            CollisionOccur = 0;
        end
        CollisionOccur_DistanceA2C2 = [CollisionOccur, distance_A2C2];        
        
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