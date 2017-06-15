classdef RCB3T1R_SinguPosAiCi
    % RCB3T1R operational mode
    % RCB3T1R is one mode with the motion of platfrom 3 Transition motion
    % and 1 rotation around z-axis    
    
    %%--------------2-RER 3T1R inverse and Forward kinematics-----------------
    %-----------------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with mode 2T2R in Six-bar case;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_3T1R_IK(obj)
    %     The FK function: [p, ABC, q1q2] = RCB_3T1R_FK(obj)
    
        %   This is written by Wan Ding in 12 Dec 2016.
    
    properties
        l1;
        l2;
        pos;
        q11q12q21q22;
    end
            
    methods
        
        function obj = RCB3T1R_SinguPosAiCi(pos,q11q12q21q22,L1,L2)% constructor
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q21q22 = q11q12q21q22;
            end
        end
        
        function [WSvalue_3T1R_SinguPosA1C1, WSvalue_3T1R_SinguPosA2C2] = RCB_3T1R_SinguPosAiCi_IK(obj)
            %%--------------2-RER 3T1R and 2T2R states inverse kinematics-----------------
            %-----------------------------------------------------------------------------
            %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
            %   mechanism with mode 2T2R in Six-bar case;
            %     The function: [EulerAngle_q11_theta, ABC, q1q2] = RCB3T1R_2T2RIK(p, l1, l2)
            %
            %   This is written by Wan Ding in 15 Nov 2016.
            %   The copyright is belong to Wan Ding.
            %
            %       p                         output position of moving platform (MP):[x, y, z]
            %       theta                     input angle of MP (C1C2 and A1A2) in planer Six-bar linkage
            %       l1                        length of the base and moving platform link
            %       l2                        length of the branch chain link
            %       x                         x position of the end-effector
            %       y                         y position of the end-effector
            %       z                         z position of the end-effector
            %       EulerAngle_q11_theta      Euler Angle(Z-Y-X)  q11, and theta: [alpha, beta, gamma, q11, theta]
            %       alpha                     rotation of the MP around Z-axis (Euler Angle, Z-Y-X)
            %       beta                      rotation of the MP around Y-axis (Euler Angle, Z-Y-X)
            %       gamma                     rotation of the MP around X-axis (Euler Angle, Z-Y-X)
            %       ABC                       The position of six points A1B1C1,A2B2C2
            %       q1q2                      The all angles q11-q15, q21-q25
            %       Ci_in_op                  Ci in frame op-xyz
            %       Ci_in_Ob                  Ci in frame Ob-xyz
            %       ki                        direction vector that is parallel to plane AiBiCi
            %       p_BinaryCode              A binary code to represent the existed of six elements: 1: existed; 0: Non-exist
            %                                 eg. [1 1 1 1 0 0] represents p = [x, y, z, alpha, [], []];
            %       q11_original              The original value of q11 when we change the value to get the 16 possible values
            %       q21_original              The original value of q21 when we change the value to get the 16 possible values
            
            % p = [x, y, z, alpha, [], []];
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
            
            %%
            for i = 1:6
                if isempty(po{i}) == 1
                    p_BinaryCode(i) = 0;
                else
                    p_BinaryCode(i) = 1;
                end
            end
            
            %% ----------------------- Calculate rotation matrix of 3T1R & 2T2R modes -----------------------
            if isequal(p_BinaryCode, [1 1 1 1 0 0]) == 1
                % 3T1R mode:  [1 1 1 1 0 0]
                % p = [x, y, z, alpha, [], []];
                % display('Notice: Inputs are:p = [x, y, z, alpha, [], []] ');
                RotationMatrix = rotz(alpha * 180 / pi);
            end
            
            %% -----------------------Calculaion of Ai & Ci in frame op-xyz-----------------------
            A1 = [0, -L1/2, 0];
            A2 = [0, L1/2, 0];
            % Ci_in_op: Ci in frame op-xyz
            C1_in_op = [0, -L1/2, 0];
            C2_in_op = [0, L1/2, 0];
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (RotationMatrix * C1_in_op')' + p(1:3);
            C2_in_Ob = (RotationMatrix * C2_in_op')' + p(1:3);
            
            %% -----------------------Judge operation mode and singularity positions: 3T1R or 2T2R mode -----------------------
            if isequal(p_BinaryCode, [1 1 1 1 0 0]) == 1 % 3T1R mode
                % Judge the singularity as 1st+5th axes of C1A1 and C2A2 overlap
                % We assume that the precision is 0.02mm (1) as industry manipulators
                if abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) > 1e-2 && abs(C2_in_Ob(2) - A2(2)) > 1e-2
                    %display('The 1st+5th axes of C1A1 and C2A2 overlapped (Please improve in the future!!!)')
                    %display('The solution can be decided by straties like(eg.2): ')
                    %display('1.Follow the angle of previous or next steps;')
                    %display('2.Transit into 2T2R, namely, q11 = -q21 = 0 ')
                    WSvalue_3T1R_SinguPosA1C1 = 1;
                    WSvalue_3T1R_SinguPosA2C2 = 0;
                    %--------------------- Assign Value --------------------
                    q11 = 0; % Anti-clockwise is positive direction. top view
                    q21 = -atan((C2_in_Ob(1) - A2(1))/(C2_in_Ob(2) - A2(2))); % Anti-clockwise is positive direction. top view
                    theta = 0;   beta = 0;    gamma = 0;
                    EulerAngle_q11_theta = [alpha, beta, gamma, q11, theta];
                    EulerAngle = EulerAngle_q11_theta(1:3);                    
                elseif abs(C1_in_Ob(1) - A1(1)) > 1e-2 && abs(C1_in_Ob(2) - A1(2)) > 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                    WSvalue_3T1R_SinguPosA1C1 = 0;
                    WSvalue_3T1R_SinguPosA2C2 = 1;
                    %--------------------- Assign Value --------------------
                    q11 = -atan((C1_in_Ob(1) - A1(1))/(C1_in_Ob(2) - A1(2))); % Anti-clockwise is positive direction. top view
                    q21 = 0; % Anti-clockwise is positive direction. top view
                    theta = 0;   beta = 0;    gamma = 0;
                    EulerAngle_q11_theta = [alpha, beta, gamma, q11, theta];
                    EulerAngle = EulerAngle_q11_theta(1:3);                     
                elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                    WSvalue_3T1R_SinguPosA1C1 = 1;
                    WSvalue_3T1R_SinguPosA2C2 = 1;
                    %--------------------- Assign Value --------------------
                    q11 = 0; % Anti-clockwise is positive direction. top view
                    q21 = 0; % Anti-clockwise is positive direction. top view
                    theta = 0;   beta = 0;    gamma = 0;
                    EulerAngle_q11_theta = [alpha, beta, gamma, q11, theta];
                    EulerAngle = EulerAngle_q11_theta(1:3);                       
                else
                    WSvalue_3T1R_SinguPosA1C1 = 0;
                    WSvalue_3T1R_SinguPosA2C2 = 0;
                    %--------------------- Assign Value -------------------
                    q11 = -atan((C1_in_Ob(1) - A1(1))/(C1_in_Ob(2) - A1(2))); % Anti-clockwise is positive direction. top view
                    q21 = -atan((C2_in_Ob(1) - A2(1))/(C2_in_Ob(2) - A2(2))); % Anti-clockwise is positive direction. top view
                    theta = 0;   beta = 0;    gamma = 0;
                    EulerAngle_q11_theta = [alpha, beta, gamma, q11, theta];
                    EulerAngle = EulerAngle_q11_theta(1:3);                        
                end    
                 %% -----------------------Calculate 16 possbile outputs for ABC, q1q2-----------------------
                    %--------------------- First step --------------------
                    %Calculate k1(parallel to plane A1B1C1),k2(parallel to plane A2B2C2),
                    % angles <A1C1,k1> and <A2C2,k2> ,lengths ||A1C1|| and ||A2C2||
                    k1 = [-sin(q11),  cos(q11), 0];
                    k2 = [ sin(q21), -cos(q21), 0];
                    A1C1 = C1_in_Ob - A1;
                    A2C2 = C2_in_Ob - A2;
                    if norm(A1C1) == 0
                        angle_A1C1_k1 = pi/2;
                        angle_A2C2_k2 = pi/2;
                    else
                        angle_A1C1_k1 = acos(dot(A1C1,k1)/(norm(A1C1)*norm(k1)));
                        angle_A2C2_k2 = acos(dot(A2C2,k2)/(norm(A2C2)*norm(k2)));
                    end
                    lengthA1C1 = norm(A1C1);
                    lengthA2C2 = norm(A2C2);
                    %--------------------- Second step --------------------
                    A1B1 = L2; B1C1 = L2;
                    A2B2 = L2; B2C2 = L2;
                    angleA1B1C1 = acos((A1B1^2 + B1C1^2 - lengthA1C1^2)/(2 * A1B1 * B1C1));
                    angleA2B2C2 = acos((A2B2^2 + B2C2^2 - lengthA2C2^2)/(2 * A2B2 * B2C2));
                    % calculate: angleA1B1C1, q13, angleB1A1C1;  angleA2B2C2, q23, angleB2A2C2
                    
                    %% ----------------------- Calculate solutions for one input  -----------------------
                    % Here are used to preserve the original value of q11
                    % and q21, which are used for i>5 due to the change of q11 and q21
                    q11_original = q11;
                    q21_original = q21;
                    
                    if q11 == 0 || q11 == pi || q11 == -pi || q21 == 0 || q21 == pi || q21 == -pi
                        IterationNumber = 5; % There exist five situations: 0/pi/-pi/2*pi/-2*pi
                    else
                        IterationNumber = 4;
                    end
                    % There exist 4/5 situations for each branch chain under the condition of single value for q13 and q23:
                    for i = 1:IterationNumber
                        
                        %--------- Situation I -----------%
                        q13 = pi - angleA1B1C1;
                        q23 = pi - angleA2B2C2;
                        
                        angleB1A1C1 = q13 / 2;
                        angleB2A2C2 = q23 / 2;
                        % calculate: q12, q14; q22, q24;
                        
                        switch i
                            %--------- Situation I-IV: input q11 -----------%
                            case 1
                                %--------- Situation I -----------%
                                q11 = q11_original;
                                q21 = q21_original;
                            case 2
                                %--------- Situation II -----------%
                                if q11_original <= 0
                                    q11 = - pi + q11_original;
                                elseif q11_original > 0
                                    q11 =  pi + q11_original;
                                end
                                if q21_original <= 0
                                    q21 = - pi + q21_original;
                                elseif q21_original > 0
                                    q21 =  pi + q21_original;
                                end
                            case 3
                                %--------- Situation III -----------%
                                if q11_original <= 0
                                    q11 = pi + q11_original;
                                elseif q11_original > 0
                                    q11 = -pi + q11_original;
                                end
                                if q21_original <= 0
                                    q21 = pi + q21_original;
                                elseif q21_original > 0
                                    q21 = -pi + q21_original;
                                end
                            case 4
                                %--------- Situation IV -----------%
                                if q11_original <= 0
                                    q11 = 2 * pi + q11_original;
                                elseif q11_original > 0
                                    q11 = -2 * pi + q11_original;
                                end
                                if q21_original <= 0
                                    q21 = 2 * pi + q21_original;
                                elseif q21_original > 0
                                    q21 = -2 * pi + q21_original;
                                end
                            case 5
                                %--------- Situation V -----------%
                                if q11_original == 0
                                    q11 = -2 * pi + q11_original;
                                elseif q11_original == pi
                                    q11 = -2 * pi;
                                elseif q11_original == -pi
                                    q11 = 2 * pi;
                                end
                                if q21_original <= 0
                                    q21 = 2 * pi + q21_original;
                                elseif q21_original == pi
                                    q21 = -2 * pi;
                                elseif q21_original == -pi
                                    q21 = 2 * pi;
                                end
                        end
                        
                        %--------- q11-q14 and q21-q24 -----------%
                        % Two modes of 3T1R and 2T2R, and the rest modes
                        if q11 == q11_original || i > 3
                            q12 = pi - angle_A1C1_k1 - angleB1A1C1;
                            q14 = (q12 + q13 + theta) - pi/2;
                        else
                            q12 = angle_A1C1_k1 - angleB1A1C1;
                            q14 = (q12 + q13) - theta - pi/2;
                        end
                        if q21 == q21_original || i > 3
                            q22 = pi - angle_A2C2_k2 - angleB2A2C2;
                            q24 = (q22 + q23) - theta - pi/2;
                        else
                            q22 = angle_A2C2_k2 - angleB2A2C2;
                            q24 = (q22 + q23 + theta) - pi/2;
                        end
                        %--------- q15 and q25 -----------%
                        q15 = - q11 + alpha;   q25 = q21 + alpha ;
                        
                        q1q2(i,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
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
                        
                    end
                    
                    %----------------------Judge the workspace and solution existence-------------------------
                    for i = 1:1:IterationNumber
                        q11 = q1q2(i,1);
                        q12 = q1q2(i,2);
                        q13 = q1q2(i,3);
                        q14 = q1q2(i,4);
                        q15 = q1q2(i,5);
                        q21 = q1q2(i,6);
                        q22 = q1q2(i,7);
                        q23 = q1q2(i,8);
                        q24 = q1q2(i,9);
                        q25 = q1q2(i,10);
                        
                        if q11 >= -2*pi && q12 >= 0 && q13 >= -pi && q14 >= -2*pi/3 && q15 >= -2*pi...
                                && q11 <= 2*pi && q12 <= pi && q13 <= pi && q14 <= 105*pi/180 && q15 <= 2*pi...                                
                                && q21 >= -2*pi && q22 >= 0 && q23 >= -pi && q24 >= -2*pi/3 && q25 >= -2*pi ...
                                && q21 <= 2*pi && q22 <= pi && q23 <= pi && q24 <= 105*pi/180 && q25 <= 2*pi...
                                && norm(C1(i,:)-C2(i,:)) - L1 < 1e-6 && isreal(q1q2) ~= 0
                            % WSvalue_3T1R_SinguPosA1C1 = 1;
                            % WSvalue_3T1R_SinguPosA2C2 = 1;
                            %Here is used to check the correctness of each exsited points
%                             q0q1q2_mat = [0, q1q2(i,:)];
%                             ReconbotANI(q0q1q2_mat);
                            break;
                        else
                            WSvalue_3T1R_SinguPosA1C1 = 0;
                            WSvalue_3T1R_SinguPosA2C2 = 0;
                            %display('Notice:The solution is incorrect, mechanism recovery to original configuration')
                        end
                    end
                 
            end
        end
        
        function [p, ABC, q1q2] = RCB_3T1R_SinguPosAiCi_FK(obj)
            %[p, ABC, q1q2] = RCB3T1RFKq11q12q21q22(q, l1, l2)
            %%-----------Inputs-------------
            % In the 3T1R mode, the FK can't get the analytical solution, so, we must solve
            % the 8-degree polynomials equations;
            %
            %%--------------2-URU 3T1R state forward kinematics-----------------
            %-------------------------------------------------------------------
            %     This function is to calculate the forward kinematics of 4 DoF 2-RER PM
            %   mechanism with mode 3T1R;
            %     In the 3T1R mode, the FK can't get the analytical solution, so, we must
            %   solve the 8-degree polynomials equations;
            %     The function: [p, ABC, q13q23] = RCB3T1RFK(q, l1, l2)
            %
            %   This is written by Wan Ding in 31 Mai 2016.
            %
            %       q                 input four angles q11 q12 q21 q22
            %       q13all            all possible angles of q13 that satisfy C1z = C2z
            %       q23all            all possible angles of q23 that satisfy C1z = C2z
            %       q13singlevalue    one of all possible angles of q13all
            %       q23singlevalue    one of all possible angles of q23all
            %       q                 input four angles q11 q12 q21 q22
            %       p                 output position of moving platform (MP)
            %       l1                length of the base and moving platform link
            %       l2                length of the branch chain link
            %       x                 x position of the end-effector
            %       y                 y position of the end-effector
            %       z                 z position of the end-effector
            %       gamma              rotation of the MP around Z-axis
            %       ABC               The position of six points A1B1C1,A2B2C2
            %       q12q23            The angles in positions B1 and B2 respectively
            %       q1q2              The all angles q11-q15, q21-q25
            %%-------------------------------------------------------------------
            
            % Variable value assignment
            q11 = obj.q11q12q21q22(1);
            q12 = obj.q11q12q21q22(2);
            q21 = obj.q11q12q21q22(3);
            q22 = obj.q11q12q21q22(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %%---------------- Calculate the value of q13 (theta13) -------------------
            
            %%-------------------the 8-degree polynomials equations-----------------------
            % J1*x^8 + J2*x^7 + J3*x^6 + J4*x^5 + J5*x^4 + J6*x^3 + J7*x^2 + J8*x + J9 = 0
            % x = tan[(q12+ q13)/2]
            % the coefficient matrix of the 8-degree polynomials equations
            % CoefficientMatrix = [J1, J2, J3, J4, J5, J6, J7, J8, J9];
            %%----------------------------------------------------------------------------
            
            % the Substitutional Variables
            F = cos(q11) * cos(q21) - sin(q11) * sin(q21);
            G = sin(q12) - sin(q22);
            
            D1 = 2 * L2 * F;
            D2 = 2 * (L2 * cos(q22) + L1 * cos(q21) + L2 * F * cos(q12));
            
            E1 = 2 * (L2 * cos(q12) + L1 * cos(q11) + L2 * F * cos(q22));
            E2 = L2 * cos(q12)^2 + 2 * (L2 * F * cos(q22) + L1 * cos(q11)) * cos(q12) + L2 * cos(q22)^2 + 2 * L1 * cos(q21) * cos(q22);
            
            H1 = L2^2;
            H2 = 2 * L2 * E1;
            H3 = 2 * L2^2 + E1^2 + 2 * L2 * E2 - D1^2;
            H4 = 2 * (L2 * E1 + E1 * E2 - D1 * D2);
            H5 = L2^2 + 2 * L2 * E2 + E2^2 - D2^2;
            
            I1 = 2 * L2^2 - D1^2;
            I2 = 2 * (L2 * E1 - D1 * D2);
            I3 = 2 * L2^2 + 2 * L2 * E2 - D2^2;
            
            %%----------------------------Simplified version----------------------------
            J1 = L2^2 * G^4 - G^2 * (I1 - I2 + I3) + H1 - H2 + H3 - H4 + H5;
            J2 = L2^2 * 8 * G^3 - 4 * G * (I1 - I2 + I3);
            J3 = L2^2 * (4 * G^4 + 24 * G^2) - (4 * (I1 - I2 + I3) + 2 * G^2 * (2 * I3 - I2)) - 4 * H1 + 2 * H2 - 2 * H4 + 4 * H5;
            J4 = L2^2 * (24 * G^3 + 32 * G) - 4 * G * (-I1 - I2 + 3 * I3);
            J5 = L2^2 * (6 * G^4 + 48 * G^2 + 16) - (8 * (I3 - I1) + 2 * G^2 * (3 * I3 - I1)) + 6 * H1 - 2 * H3 + 6 * H5;
            J6 = L2^2 * (24 * G^3 + 32 * G) - 4 * G * (-I1 + I2 + 3 * I3);
            J7 = L2^2 * (4 * G^4 + 24 * G^2) - (4 * (I1 + I2 + I3) + 2 * G^2 * (I2 + 2 * I3)) - 4 * H1 - 2 * H2 + 2 * H4 + 4 * H5;
            J8 = L2^2 * 8 * G^3 - 4 * G * (I1 + I2 + I3);
            J9 = L2^2 * G^4 - G^2 * (I1 + I2 + I3) + H1 + H2 + H3 + H4 + H5;
            
            % the coefficient matrix of the 8-degree polynomials equation
            CoefficientMatrix8 = [J1, J2, J3, J4, J5, J6, J7, J8, J9];
            
            % calculate the root of the 8-degree polynomials equation
            x = roots(CoefficientMatrix8);
            
            % get the value of real number (+/-)
            x = x(imag(x) == 0);
            NumRealq13 = length (x);
            
            % the final value of "+/-theta13"
            % the programm should judge the sign of the value
            
            q13all = 2 * atan(x) - q12 ;
            %  q13all*180/pi
            
            %% ------------------Obtain all of the correct values and assign to q13q23-------------------------
            % numbers of i and j are used to count the possible values that satisfy the
            % condiation of C1z = C2z; And then, assign all the possible values to
            % matrix q13q23;
            
            i = 1;
            for Numq13 = 1:NumRealq13
                
                q13SingleValue = q13all(Numq13);
                j = 0;
                %% -------------- Calculate the value of q23 (theta23) ----------------
                % Using the following function to calculate the value of q23
                % L2 * cos(q23)^2 + (D1 * cos(q13) + D2) * cos(q23) + (L2 * cos(q13)^2 + E1 * cos(q13) + E2) = 0
                % the coefficient matrix of the 2-degree polynomials equations
                K1 = L2;
                K2 = D1 * cos(q12 + q13SingleValue) + D2;
                K3 = L2 * cos(q12 + q13SingleValue)^2 + E1 * cos(q12 + q13SingleValue) + E2;
                
                % the coefficient matrix of the 2-degree polynomials equation
                CoefficientMatrix2 = [K1, K2, K3];
                
                % calculate the root of the 2-degree polynomials equation
                cosq23 = roots(CoefficientMatrix2);
                
                % get the value of real number (+/-)
                cosq23 = cosq23(imag(cosq23) == 0);
                NumRealq23 = length (cosq23);
                
                % the final value of "+/-theta13"
                q23all = acos(cosq23) - q22;
                %q23all * 180 / pi
                
                for Numq23 = 1:NumRealq23
                    
                    q23SingleValue = q23all(Numq23);
                    %% -----------------------Get the output values of Moving Platform-----------------------
                    %%--------------------Calculate the position of Ai Bi Ci------------------
                    A1 = [0, -L1/2, 0];
                    B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                    
                    A2 = [0, L1/2, 0];
                    B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
                    C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                    %%------------------------------------------------------------------------

                    % Get rid of the undesired points
                    if abs(C1(3) - C2(3)) > 1e-5
                        continue;
                    end
                    
                    % assign the values to q13q23
                    q13 = q13SingleValue;
                    q23 = q23SingleValue;
                    
                    % Calculate the center point of moving platform
                    p(i+j,1:3) = (C1 + C2) / 2;
                    C1C2 = C2 - C1;
                    yaxis = [0, 1, 0];
                    % Judge the direction of the rotation around Z-axis
                    directionC1C2Xxaxis = cross(C1C2, yaxis);
                    if directionC1C2Xxaxis(3) < 0
                        gamma(i+j) = -acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
                    else
                        gamma(i+j) = acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
                    end
                    gamma(i+j) * 180 / pi;
                    p(i+j,4:6) = [0, 0, gamma(i+j)];
                    ABC(i+j,:) = [A1, B1, C1, A2, B2, C2];
                    
                    %%-------------------------q14q15 and q24q25------------------------------
                    % Calculate the angles of q14q15 and q24q25
                    q14 = pi - q12 - q13;% Actually, it should be:q14 = q12 + q13 - pi/2;
                    q15 = -q11 - gamma(i+j);
                    q24 = pi - q22 - q23;%Actually, it should be:q14 = q22 + q23 - pi/2;
                    q25 = q21 - gamma(i+j);
                    %%------------------------------------------------------------------------
                    %%-------------------------q11-q15 and q21-q25------------------------------
                    q1q2(i+j,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                    
                    j = j + 1;
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
                             %grid on;
                             %axis equal;
                             xlabel('x');
                             ylabel('y');
                             zlabel('z');
                             axis equal;
                end
                i = i + j;
            end
            
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
            
            %%Here is used to judge the existance of the solution
            if i == 1
                display('No solution for this input, Calculation process is stopped');
                p = [];
                ABC = [];
                q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
            end
        end
               
    end
    
end

