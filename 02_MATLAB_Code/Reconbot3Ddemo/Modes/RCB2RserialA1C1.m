classdef RCB2RserialA1C1
    % RCB2RserialA1C1 operational mode
    % RCB2RserialA1C1 is one mode with the motion of platfrom 2 rotation motions around xy-axis        
    %%------------2-RER 2R serialA1C1 inverse and Forward kinematics--------------
    %--------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with mode 2R in one serial chain A1B1C1;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(obj)
    %     The FK function: [p, ABC, q1q2] = RCB_2R_serialA1C1_FK(obj)    
    %     This is written by Wan Ding in 13 Dec 2016.
    %---------------------------------------------------------------------
    
    properties
        l1;
        l2;
        pos;
        q11q12q22q13;
    end
    
    methods
        function obj = RCB2RserialA1C1(pos,q11q12q22q13,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q22q13 = q11q12q22q13;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(obj)
            % p = [x, y, [], [], [], []]; y < 0
            L1 = obj.l1;
            L2 = obj.l2;
            po = obj.pos;
            for i = 1:3
                if isempty(po{i}) == 0
                    p(i) = po{i};
                end
            end
            alpha = po{4};
            beta  = po{5};
            gamma = po{6};
            
            switch length(po)
                case 7 % Serial A1C1 & A2C2
                    if p(2) < 0
                        q12 = po{7};
                    elseif p(2) > 0
                        q22 = po{7};
                    end
                case 10
                    q11 = po{7};
                    q12 = po{8};
                    q21 = po{9};
                    q22 = po{10};
            end
            
            %% -----------------------Calculaion of Ai & Ci in frame op-xyz-----------------------
            A1 = [0, -L1/2, 0];
            A2 = [0, L1/2, 0];
            
            %%
            for i = 1:6
                if isempty(po{i}) == 1
                    p_BinaryCode(i) = 0;
                else
                    p_BinaryCode(i) = 1;
                end
            end
            
            %% ----------------------- Calculate rotation matrix according to inputs -----------------------
            if isequal(p_BinaryCode, [1 1 0 0 0 0]) == 1
                %% ----------------------- Calculate rotation matrix of two 2R modes -----------------------
                if p(2) < 0
                    % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
                    % p = [x, y, [], [], [], []]; y < 0
                    display('Notice: Inputs are:p = [x, y, [], [], [], []]; y < 0');
                    display('Four-bar linkage with Serial Chain A1C1');
                    % 1.
                    q11 = - atan(p(1)/p(2));
                    Oboppie = sqrt(p(1)^2 + p(2)^2);
                    A1Ob = L1/2;
                    A1op = L1/2;
                    % 2.
                    A1oppie = sqrt(Oboppie^2 + A1Ob^2 - 2 * Oboppie * A1Ob * cos(q11));
                    % 3.
                    zop = sqrt(A1op^2 - A1oppie^2);
                    % Judge the exisitence of soultion
                    if isreal(zop) == 0
                        display('There exist no solution')
                        return;
                    end
                    p(3) = zop;
                    A1pieOb = A1Ob * cos(q11);
                    A1pieoppie =  A1pieOb - Oboppie;
                    % 4.
                    if A1pieoppie > 0
                        theta = atan(zop / A1pieoppie);
                    elseif A1pieoppie ==0
                        theta = pi/2;
                    else
                        theta = atan(zop / A1pieoppie) + pi;
                    end
                    % 5.
                    zC2 = 2 * zop;
                    vector_Oboppie = [p(1),p(2),0];
                    C2pie = A2 + 2 * vector_Oboppie;
                    C2 = [C2pie(1), C2pie(2), zC2];
                    % 6.
                    vector_A2C2pie = C2pie - A2;
                    vector_A2C2 = C2 - A2;
                    angleC2pieA2C2 = atan(zC2 / norm(vector_A2C2pie));
                    angleA2B2C2 = acos((2 * L2^2 - norm(vector_A2C2)^2) / (2 * L2^2));
                    %-------------------q11-q15, q21-q25------------------------------
                    q21 = q11;
                    angleA1B1C1 = 0;
                end
                % ----------------------Calculate the Euler angle--------------------------
                u_RotationAxis = [cos(q11), sin(q11), 0];
                r = [u_RotationAxis, theta];
                %-- m=vrrotvec2mat(r):Convert rotation from axis-angle to matrix representation--
                Matrix_from_axis_angle = vrrotvec2mat(r);
                %-- eul = rotm2eul(rotm): Extract Euler angles from homogeneous transformation--
                EulerAngle = rotm2eul(Matrix_from_axis_angle);
                
                EulerAngle_q11_theta = [EulerAngle, q11, theta];
            end
            
            %% ----------------------- Calculate one solutions for one input  -----------------------
            if p(1) == 0 && p(2) == 0 && p(3) == 0
                IterationNumber = 1;
                q13 = pi;
                q14 = - q12;
                q15 = - q11;
                
                q23 = pi;
                q24 = - q22;
                q25 = q21;
                q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
            else
                % Here are used to preserve the original value of q11
                % and q21, which are used for i>5 due to the change of q11 and q21
                q11_original = q11;
                q21_original = q21;
                if (q11 == 0 && q21 ~= 0) || (q11 ~= 0 && q21 == 0)
                    % For situation: q11 = 0/pi/-pi, q21 = q21/[(pi+q21)/(-pi+q21)], there exist 6 combinations, with 24 solutions
                    % For situation: q11 = q11/[(pi+q11)/(-pi+q11)], q21 = 0/pi/-pi, there exist 6 combinations, with 24 solutions
                    IterationNumber = 24;
                elseif q11 == 0 && q21 == 0
                    % For situation: q11 = 0/pi/-pi, q21 = 0/pi/-pi,there exist 9 situations with 36 situations
                    IterationNumber = 36;
                else
                    % For situation: q11 = q11/[(pi+q11)/(-pi+q11)], q21 = q21/[(pi+q21)/(-pi+q21)],there exist 4 situations
                    IterationNumber = 16;
                end
                % There exist 16 situations:
                for i = 1:IterationNumber
                    switch rem(i,4)
                        %--------- Situation I-IV: input q11 -----------%
                        case 1
                            %--------- Situation I -----------%
                            q13 = pi - angleA1B1C1;
                            q23 = pi - angleA2B2C2;
                        case 2
                            %--------- Situation II -----------%
                            q13 = angleA1B1C1 - pi;
                            q23 = pi - angleA2B2C2;
                        case 3
                            %--------- Situation III -----------%
                            q13 = pi - angleA1B1C1;
                            q23 = angleA2B2C2 - pi;
                        case 0
                            %--------- Situation IV -----------%
                            q13 = angleA1B1C1 - pi;
                            q23 = angleA2B2C2 - pi;
                    end
                    angleB1A1C1 = q13 / 2;
                    angleB2A2C2 = q23 / 2;
                    % calculate: q12, q14; q22, q24;
                    
                    % For 36 situation: q11 = 0/pi/-pi, q21 = 0/pi/-pi,there exist 9 situations with 36 situations
                    % [0,0],[0,pi],[0,-pi], [pi,0],[pi,pi],[pi,-pi], [-pi,0],[-pi,pi],[-pi,-pi]
                    
                    % For 24 situation: q11/[(pi+q11)/(-pi+q11)], q21 = q21/[(pi+q21)/(-pi+q21)],there exist 4 situations
                    % [q11, q21], [q11, (pi+q21)/(-pi+q21)], [(pi+q11)/(-pi+q11), q21], [(pi+q11)/(-pi+q11), (pi+q21)/(-pi+q21)]
                    
                    % -------------------------------------------------------------------
                    % ----------------------- Logic in this part  -----------------------
                    % -------------------------------------------------------------------
                    % 1. For 16 solutions, IterationNumber = 16;
                    %   1.1 Steps i >= 1 && i < 17:
                    % 2. For 36 solutions, IterationNumber = 36;
                    %   3.1 Steps i >= 1 && i < 17:
                    %   3.1 Steps i >= 17 && i < 25: q11 == 0; the 'if" part;
                    %   3.2 Steps i >= 25 && i < 33: q21 == 0; the 'if" part;
                    %   3.3 Steps i >= 33 && i < 37: q11 == 0 && q21 == 0; the 'if" part;
                    
                    if i < 5  % [q11, q21] or [0, 0]
                        
                    elseif i >= 5 && i < 9  % [(pi+q11)/(-pi+q11), q21] or [pi, 0]
                        if q11_original <= 0 && i == 5
                            q11 = pi + q11_original;
                        elseif q11_original > 0 && i == 5
                            q11 = -pi + q11_original;
                        end
                        q21 = q21_original;
                    elseif i >= 9 && i < 13  % [q11, (pi+q21)/(-pi+q21)] or [0, pi]
                        if q21_original <= 0 && i == 9
                            q21 = pi + q21_original;
                        elseif q21_original > 0 && i == 9
                            q21 = -pi + q21_original;
                        end
                        q11 = q11_original;
                    elseif i >= 13 && i < 17 % [(pi+q11)/(-pi+q11), (pi+q21)/(-pi+q21)] or [pi, pi]
                        if q11_original <= 0 && i == 13
                            q11 = pi + q11_original;
                        elseif q11_original > 0 && i == 13
                            q11 = -pi + q11_original;
                        end
                        if q21_original <= 0 && i == 13
                            q21 = pi + q21_original;
                        elseif q21_original > 0 && i == 13
                            q21 = -pi + q21_original;
                        end
                    elseif i >= 17 && i < 21 % [-pi, q21] or [-pi, 0] or [0, -pi]
                        if q11_original == 0 && i == 17% [-pi, q21] or [-pi, 0]
                            q11 = -pi + q11_original;
                            q21 = q21_original;
                        elseif q11_original ~= 0 && q21_original == 0 && i == 17  % [q11, -pi] or [0, -pi]
                            q21 = -pi + q21_original;
                            q11 = q11_original;
                        end
                    elseif i >= 21 && i < 25 && i == 21% [-pi, (pi+q21)/(-pi+q21)] or [-pi, pi] or [pi, -pi]
                        if q11_original == 0 % [-pi, (pi+q21)/(-pi+q21)] or [-pi, pi]
                            q11 = -pi + q11_original;
                            if q21_original <= 0
                                q21 = pi + q21_original;
                            elseif q21_original > 0
                                q21 = -pi + q21_original;
                            end
                        elseif q11_original ~= 0 && q21_original == 0 && i == 21% [(pi+q11)/(-pi+q11), pi] or [pi, -pi]
                            q21 = -pi + q21_original;
                            if q11_original <= 0
                                q11 = pi + q11_original;
                            elseif q21_original > 0
                                q11 = -pi + q11_original;
                            end
                        end
                    elseif i >= 25 && i < 29 % [0, -pi]
                        if  q21_original == 0 && i == 25
                            q21 = -pi + q21_original;
                            q11 = q11_original;
                        end
                    elseif i >= 29 && i < 33 % [pi, -pi]
                        if  q21_original == 0 && i == 29
                            q21 = -pi + q21_original;
                            if q11_original <= 0
                                q11 = pi + q11_original;
                            elseif q21_original > 0
                                q11 = -pi + q11_original;
                            end
                        end
                    elseif i >= 33 && i < 37 % [-pi, -pi]
                        if q11_original == 0 && q21_original == 0 && i == 33
                            q11 = -pi + q11_original;
                            q21 = -pi + q21_original;
                        end
                    end
                    %--------- q11-q14 and q21-q24 -----------%
                    % Two modes of 3T1R and 2T2R, and the rest modes
                    if q11 == q11_original
                        %q12 is a input;
                        q14 = (q12 + q13 + theta) - pi/2;
                    else
                        q14 = (q12 + q13) - theta - pi/2;
                    end
                    if q21 == q21_original
                        q22 = pi - angleC2pieA2C2 - q23/2;
                        q24 = (q22 + q23) - theta - pi/2;
                    else
                        q22 = angleC2pieA2C2 - q23/2;
                        q24 = (q22 + q23 + theta) - pi/2;
                    end
                    %------------ q15 and q25 -----------%
                    q15 = q11;           q25 = q21;
                    
                    q1q2(i,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                end
            end
            
            for i = 1:1:IterationNumber
                %%-----------------------Get the output values of Moving Platform-----------------------
                %%--------------------Calculate the position of Ai Bi Ci------------------
                A1(i,:) = [0, -L1/2, 0];
                B1(i,:) = [L2 * cos(q1q2(i,2)) * sin(q1q2(i,1)), -L1/2 - L2 * cos(q1q2(i,2)) * cos(q1q2(i,1)), L2 * sin(q1q2(i,2))];
                C1(i,:) = [L2 * (cos(q1q2(i,2)) + cos(q1q2(i,2) + q1q2(i,3))) * sin(q1q2(i,1)), -L1/2 - L2 * (cos(q1q2(i,2))...
                    + cos(q1q2(i,2) + q1q2(i,3))) * cos(q1q2(i,1)), L2 * (sin(q1q2(i,2)) + sin(q1q2(i,2) + q1q2(i,3)))];
                
                A2(i,:) = [0, L1/2, 0];
                B2(i,:) = [- L2 * cos(q1q2(i,7)) * sin(q1q2(i,6)), L1/2 + L2 * cos(q1q2(i,7)) * cos(q1q2(i,6)), L2 * sin(q1q2(i,7))];
                C2(i,:) = [- L2 * (cos(q1q2(i,7)) + cos(q1q2(i,7) + q1q2(i,8))) * sin(q1q2(i,6)), L1/2 + L2 * (cos(q1q2(i,7))...
                    + cos(q1q2(i,7) + q1q2(i,8))) * cos(q1q2(i,6)), L2 * (sin(q1q2(i,7)) + sin(q1q2(i,7) + q1q2(i,8)))];
                %%------------------------------------------------------------------------
                
                %----------------------Position of A1-C1 and A2-C2-------------------------
                ABC(i,:) = [A1(i,:), B1(i,:), C1(i,:), A2(i,:), B2(i,:), C2(i,:)];
                
                %----------------------Judge the workspace and solution existence-------------------------
                if norm(C1(i,:)-C2(i,:)) - L1 > 1e-6 || isreal(q1q2) == 0
                    display('Notice:The solution is incorrect, mechanism recovery to original configuration')
                    %q1q2(i+1,:) = [0, pi/3, pi/3, pi/3, 0, 0, pi/3, pi/3, pi/3, 0];
                    break
                end
                
                RCB_ABCplot3;
                hold off
            end
            %------------------------------------------------------------------------
            p = [po{1}, po{2}, po{3}, EulerAngle];
        end
        
        function [p, ABC, q1q2] = RCB_2R_serialA1C1_FK(obj)
            %%------------Inputs-------------
            % Variable value assignment
            q11 = obj.q11q12q22q13(1);
            q12 = obj.q11q12q22q13(2);
            q22 = obj.q11q12q22q13(3);
            q13 = obj.q11q12q22q13(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %% -----------------------Basic calculaion of planar four-bar linkage-----------------------
            q21 = q11;
            l1pie = L1 * cos(q11);
            
            %--------------------- Parallelogram ------------------
            lA1B2pie = sqrt(l1pie^2 + L2^2 + 2 * l1pie * L2 * cos(q22));
            thetaB2A1A2pie = acos((l1pie^2 + lA1B2pie^2 - L2^2) / (2 * l1pie * lA1B2pie));
            
            angleC2A1A2pie = 2 * thetaB2A1A2pie;
            angleC2B2A2pie = 2 * q22 - angleC2A1A2pie;
            
            %According to the geometry relation, that lA1A2pie = lA1C2pie, lA2B2pie = lB2C2pie;
            q24 = q22;
            q23 = pi - angleC2B2A2pie;
            q14 = - (angleC2A1A2pie + q12);
            
            %% -----------------------Get the output values of Moving Platform-----------------------
            %%--------------------Calculate the position of Ai Bi Ci------------------
            A1 = [0, -L1/2, 0];
            B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
            C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
            
            A2 = [0, L1/2, 0];
            B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
            C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
            %%------------------------------------------------------------------------
            
            %norm(C1-C2)
            %-------------------------q15 = q25------------------------------
            % Calculate the angles of q15
            q15 = - q11;
            q25 = q21;
            %-------------------Transform into angle-------------------
            %q15_Angle = q15 * 180 / pi;
            
            %----------------------Position of A1-C1 and A2-C2-------------------------
            ABC = [A1, B1, C1, A2, B2, C2];
            %-------------------------q11-q15 and q21-q25------------------------------
            q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
            %-------------Calculate the center point of moving platform----------------
            p = (C1 + C2) / 2;
            vectorC1C2 = C2 - C1;
            %%------------------------------------------------------------------------
            
            %% ----------------------Calculate the RPY angle--------------------------
            theta = pi - (q12 + q13 + q14);
            u_RotationAxis = [cos(q11), sin(q11), 0];
            r = [u_RotationAxis, theta];
            %-- m=vrrotvec2mat(r):Convert rotation from axis-angle to matrix representation--
            Matrix_from_axis_angle = vrrotvec2mat(r);
            %-- tform = rotm2tform(rotm): Convert rotation matrix to homogeneous transformation--
            Tform_from_axis_angle = rotm2tform(Matrix_from_axis_angle);
            %-- eul = tform2eul(tform): Extract Euler angles from homogeneous transformation--
            eul_alpha_beta_gamma = tform2eul(Tform_from_axis_angle);
            
            p(4:6) = eul_alpha_beta_gamma;
            %% --------------------Plot the mechanism Ai Bi Ci------------------
            PA1B1C1x = [A1(1), B1(1), C1(1)];
            PA1B1C1y = [A1(2), B1(2), C1(2)];
            PA1B1C1z = [A1(3), B1(3), C1(3)];
            plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
            
            PA2B2C2x = [A2(1), B2(1), C2(1)];
            PA2B2C2y = [A2(2), B2(2), C2(2)];
            PA2B2C2z = [A2(3), B2(3), C2(3)];
            plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
            
            PC1C2x = [C1(1), C2(1)];
            PC1C2y = [C1(2), C2(2)];
            PC1C2z = [C1(3), C2(3)];
            plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
            
            PA1A2x = [A1(1), A2(1)];
            PA1A2y = [A1(2), A2(2)];
            PA1A2z = [A1(3), A2(3)];
            plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
            
            %----------------- plot xyz axes of base point --------------
            x_axis = [0.5 0 0];
            y_axis = [0 0.5 0];
            z_axis = [0 0 0.5];
            OP= [0 0 0];
            xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
            i = 1:2;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
            i = 3:4;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
            i = 5:6;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
            %-----------------------------------------------------------
            %------------------plot xyz axes of Moving Platform----------------
            xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(Matrix_from_axis_angle * transpose(xyz));
            i = 1:2;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
            hold on;
            axis equal;
            i = 3:4;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
            i = 5:6;
            plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
            %----------------------------------------------
            
            %grid on;
            %axis equal;
            xlabel('x');
            ylabel('y');
            zlabel('z');
            axis equal;
            %%------------------------------------------------------------------------
        end       
        
    end
    
end

