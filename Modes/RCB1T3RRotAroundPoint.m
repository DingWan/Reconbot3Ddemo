classdef RCB1T3RRotAroundPoint
    % RCB1T2RRotAroundPoint operational mode
    % RCB1T2RRotAroundPoint is actually not a operational mode due to the
    % motion composed of 3T1R-2T2R-Fivebar-Threebar.
    % Here, we still call it 1T2R with the motion of platfrom 1 Transition motion along z-axis and 2 rotation around xyz-axis        
    %%------------2-RER 1T3R RotAroundPoint inverse kinematics--------------
    %--------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with 1T3R, Rotate Around Point [0, 0 , z] case;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_1T3R_RotAroundPoint_IK(obj)   
    %     This is written by Wan Ding in 13 Dec 2016.
    %---------------------------------------------------------------------
    
    properties
        l1;
        l2;
        pos;
        q11q12q14q23;
    end
        
    methods
        
        function obj = RCB1T3RRotAroundPoint(pos,q,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q = q;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_1T3R_RotAroundPoint_IK(obj)
            % Mechanism rotate around point p(1:3):  [1 1 1 0 1 1]
            % p = [[], [], z, [], beta, gamma]
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
            % Ci_in_op: Ci in frame op-xyz
            C1_in_op = [0, -L1/2, 0];
            C2_in_op = [0, L1/2, 0];
            
            %%
            for i = 1:6
                if isempty(po{i}) == 1
                    p_BinaryCode(i) = 0;
                else
                    p_BinaryCode(i) = 1;
                end
            end
            
            %% ----------------------- Calculate rotation matrix of 3T1R & 2T2R modes -----------------------
            if isequal(p_BinaryCode, [1 1 1 0 0 0]) == 1
                % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
                % p = [0, 0, 0, [], [], []]
                display('Notice: Inputs are:p = [0, 0, 0, [], [], []]');
                display('Moving platform is fixed! p = [0, 0, 0, 0, 0, 0]');
                alpha = 0;
                beta = 0;
                gamma = 0;
                theta = 0;
                EulerAngle = [alpha, beta, gamma];
                EulerAngle_q11_theta = [EulerAngle, q11, theta];
                RotationMatrix = eul2rotm(EulerAngle);
            else
                % Euler angle IK
                if isequal(p_BinaryCode, [0 0 1 0 1 1]) == 1
                    % Mechanism rotate around point p(1:3):  [1 1 1 0 1 1]
                    % p = [[], [], z, [], beta, gamma]; x = y = 0
                    p(1) = 0;
                    p(2) = 0;
                    display('Notice: Inputs are:p = [0, 0, z, [], beta, gamma]');
                    display('Mechanism rotate around point p(1:3)');
                    q11 = []; % inputs: beta, gamma;
                end
                [EulerAngle_q11_theta] = EulerAngles_beta_gamma_q11_IK(beta, gamma, q11);
            end
            
            %% --------Choose one of the correct "EulerAngle_q11_theta" ----------
            for i = 1:length(EulerAngle_q11_theta(:,1))
                RotationMatrix = eul2rotm(EulerAngle_q11_theta(i,1:3));
                alpha = EulerAngle_q11_theta(i,1);
                beta  = EulerAngle_q11_theta(i,2);
                gamma = EulerAngle_q11_theta(i,3);
                EulerAngle = EulerAngle_q11_theta(i,1:3);
                % Ci_in_Ob: Ci in frame Ob-xyz
                C1_in_Ob = (RotationMatrix * C1_in_op')' + p(1:3);
                C2_in_Ob = (RotationMatrix * C2_in_op')' + p(1:3);
                
                q11 = EulerAngle_q11_theta(i,4);
                theta = EulerAngle_q11_theta(i,5);
                q21 = q11;
                
                if (C2_in_Ob(3) -  C1_in_Ob(3)) * theta >= 0
                    break;
                else
                    continue;
                end
            end
            
            %% -----------------------Calculate eight possbile outputs for ABC(1:8), q1q2(1:8)-----------------------
            %--------------------- First step --------------------
            %Calculate k1(parallel to plane A1B1C1),k2(parallel to plane A2B2C2),
            % angles <A1C1,k1> and <A2C2,k2> ,lengths ||A1C1|| and ||A2C2||
            k1 = [-sin(q11), cos(q11), 0];
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
                    % 2. For 24 solutions, IterationNumber = 24;
                    %   2.1 Steps i >= 1 && i < 17:
                    %   2.1 Steps i >= 17 && i < 25: q11 == 0 && q21 ~= 0; the 'if" part
                    %   2.2 Steps i >= 17 && i < 25: q21 == 0 && q11 ~= 0; the "else" part;
                    % 3. For 36 solutions, IterationNumber = 36;
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
                        q12 = pi - angle_A1C1_k1 - angleB1A1C1;
                        q14 = (q12 + q13 + theta) - pi/2;
                    else
                        q12 = angle_A1C1_k1 - angleB1A1C1;
                        q14 = (q12 + q13) - theta - pi/2;
                    end
                    if q21 == q21_original
                        q22 = pi - angle_A2C2_k2 - angleB2A2C2;
                        q24 = (q22 + q23) - theta - pi/2;
                    else
                        q22 = angle_A2C2_k2 - angleB2A2C2;
                        q24 = (q22 + q23 + theta) - pi/2;
                    end
                    %--------- q15 and q25 -----------%
                    q15 = - q11;          q25 = q21;
                    
                    q1q2(i,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                end
                
            end
            j = 0;
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
                
                %C1(i,:) + C2(i,:)
                if abs(C1(i,1) + C2(i,1))/2 <= 1e-8 && abs(C1(i,2) + C2(i,2))/2 <= 1e-8
                    %----------------------Position of A1-C1 and A2-C2-------------------------
                    j = j + 1;
                    q1q2(j,:) = q1q2(i,:);
                    ABC(j,:) = [A1(i,:), B1(i,:), C1(i,:), A2(i,:), B2(i,:), C2(i,:)];
                else
                    continue;
                end
                
                %----------------------Judge the workspace and solution existence-------------------------
                if norm(C1(i,:)-C2(i,:)) - L1 > 1e-6 || isreal(q1q2) == 0
                    display('Notice:The solution is incorrect, mechanism recovery to original configuration')
                    %q1q2(i+1,:) = [0, pi/3, pi/3, pi/3, 0, 0, pi/3, pi/3, pi/3, 0];
                    break
                end
                RCB_ABCplot3;
                %hold off
            end
            %------------------------------------------------------------------------
            p = [po{1}, po{2}, po{3}, EulerAngle];
        end
        
    end
end

