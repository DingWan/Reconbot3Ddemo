classdef RCB1T2RRotAroundPoint
    % RCB1T2RRotAroundPoint operational mode
    % RCB1T2RRotAroundPoint is actually not a operational mode due to the
    % motion composed of 3T1R-2T2R-Fivebar-Threebar.
    % Here, we still call it 1T2R with the motion of platfrom 1 Transition motion along z-axis and 2 rotation around xyz-axis        
    %%------------2-RER 1T2R RotAroundPoint inverse kinematics--------------
    %--------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with 1T2R, Rotate Around Point [0, 0 , z] case;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(obj)   
    %     This is written by Wan Ding in 13 Dec 2016.
    %---------------------------------------------------------------------
    
    properties
        l1;
        l2;
        pos;
        q11q12q14q23;
    end
        
    methods
        
        function obj = RCB1T2RRotAroundPoint(pos,q11q12q14q23,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q14q23 = q11q12q14q23;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_1T2R_RotAroundPoint_IK(obj)
            % Mechanism rotate around point p(1:3):  [1 1 1 0 1 1]
            % p = [[], [], z, [], beta, gamma]
            L1 = obj.l1;
            L2 = obj.l2;
            po = obj.pos;
            q11q12q14q23 = obj.q11q12q14q23;
            for i = 1:3
                if isempty(po{i}) == 0
                    p(i) = po{i};
                else
                    p(i) = 0;
                end
            end
            
            q11_Axis  = po{5};
            theta = po{6};
            
            switch length(po)
                case 7 % Serial A1C1 & A2C2
                    if p(2) < 0
                        q12 = po{7};
                    elseif p(2) > 0
                        q22 = po{7};
                    end
                case 8 % SingularityPositions 3T1R
                    if isempty(po{7}) == 0 && isempty(po{8}) == 1
                        q11_SP_A1C1overlap = po{7};
                    elseif isempty(po{7}) == 1 && isempty(po{8}) == 0
                        q21_SP_A2C2overlap = po{8};
                    elseif isempty(po{7}) == 0 && isempty(po{8}) == 0
                        q11_SP_A1C1_A2C2_overlap = po{7};
                        q21_SP_A1C1_A2C2_overlap = po{8};
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
                
                WSvalue_2T2R_SinguPosA1C1 = 1;
                WSvalue_1T2R_SinguPosA2C2 = 1;
            else
                
                % Euler angle IK
                if isequal(p_BinaryCode, [0 0 1 0 1 1]) == 1
                    % Mechanism rotate around point p(1:3):  [1 1 1 0 1 1]
                    % p = [[], [], z, [], beta, gamma]; x = y = 0
                    if abs(theta) < 1e-8 || po{3} < 0
                        WSvalue_1T2R = 0;
                        po{1} = 0;
                        po{2} = 0;
                        q11 = q11_SP_A1C1_A2C2_overlap;
                        q21 = q21_SP_A1C1_A2C2_overlap;
                    else
                        po{1} = 0;
                        po{2} = 0;
                    end                   
                end
                [EulerAngle_q11_theta] = EulerAngles_theta_q11_IK(theta, q11_Axis);
                %%--------Choose one of the correct "EulerAngle_q11_theta" ----------
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
                
                po{4} = alpha;
                po{5} = beta;
                po{6} = gamma;
                name = '2T2R-1T2RRotAroundPoint';
                fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                    name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi);
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
                WSvalue_2T2R_SinguPosA1C1 = 1;
                WSvalue_2T2R_SinguPosA2C2 = 1;
                IterationNumber = 1;
                q11q12q14q23 = q11q12q14q23;
                q13 = pi;
                q14 = q11q12q14q23(2);
                q15 = q11q12q14q23(1);
                
                q23 = pi;
                q24 = q11q12q14q23(4);
                q25 = q11q12q14q23(3);
                q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
            else
                % Here are used to preserve the original value of q11
                % and q21, which are used for i>5 due to the change of q11 and q21
                q11_original = q11;
                q21_original = q21;
                if q11 == 0 || q11 == pi || q11 == -pi || q21 == 0 || q21 == pi || q21 == -pi
                    IterationNumber = 5; % There exist five situations: 0/pi/-pi/2*pi/-2*pi
                else
                    IterationNumber = 4;
                end
                % There exist 4/5 situations:
                jA1C1 = 0; % NumberofFeasibleSolutionA1C1 = 0;
                jA2C2 = 0; % NumberofFeasibleSolutionA2C2 = 0;
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
                        q14 = pi/2 - (q12 + q13 + theta);
                    else
                        q12 = angle_A1C1_k1 - angleB1A1C1;
                        q14 = pi/2 + theta - (q12 + q13);
                    end
                    if q21 == q21_original || i > 3
                        q22 = pi - angle_A2C2_k2 - angleB2A2C2;
                        q24 = pi/2 + theta - (q22 + q23);
                    else
                        q22 = angle_A2C2_k2 - angleB2A2C2;
                        q24 = pi/2 - (q22 + q23 + theta);
                    end
                    %--------- q15 and q25 -----------%
                    q15 = q11;           q25 = q21;
                    
                    q1q2(i,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                    
                    %------------------Judge the workspace and solution existence of A1C1-------------------------
                    %---------------------------Position of A1-C1 ----------------------------
                    if q11 >= -2*pi && q12 >= 0 && q13 >= -pi && q14 >= -105*pi/180 && q15 >= -2*pi...
                            && q11 <= 2*pi && q12 <= pi && q13 <= pi && q14 <= 105*pi/180 && q15 <= 2*pi...
                            && isreal(q1q2(i,1:5)) ~= 0
                        jA1C1 = jA1C1 + 1;
                        %%-----------------Get the output values of Moving Platform-----------------------
                        %%--------------------Calculate the position of Ai Bi Ci------------------
                        A1(jA1C1,:) = [0, -L1/2, 0];
                        B1(jA1C1,:) = [L2 * cos(q1q2(i,2)) * sin(q1q2(i,1)), -L1/2 - L2 * cos(q1q2(i,2)) * cos(q1q2(i,1)), L2 * sin(q1q2(i,2))];
                        C1(jA1C1,:) = [L2 * (cos(q1q2(i,2)) + cos(q1q2(i,2) + q1q2(i,3))) * sin(q1q2(i,1)), -L1/2 - L2 * (cos(q1q2(i,2))...
                            + cos(q1q2(i,2) + q1q2(i,3))) * cos(q1q2(i,1)), L2 * (sin(q1q2(i,2)) + sin(q1q2(i,2) + q1q2(i,3)))];
                        %%------------------------------------------------------------------------
                        
                        if norm(C1(jA1C1,:) - C1_in_Ob) <= 1e-6
                            q1(jA1C1,1:5) = q1q2(i,1:5);
                            A1B1C1(jA1C1,:) = [A1(jA1C1,:), B1(jA1C1,:), C1(jA1C1,:)];
                        end
                    end
                    
                    %%------------------------------------------------------------------------
                    %---------------------------Position of A2-C2 ----------------------------
                    if q21 >= -2*pi && q22 >= 0 && q23 >= -pi && q24 >= -105*pi/180 && q25 >= -2*pi ...
                            && q21 <= 2*pi && q22 <= pi && q23 <= pi && q24 <= 105*pi/180 && q25 <= 2*pi...
                            && isreal(q1q2(i,6:10)) ~= 0
                        jA2C2 = jA2C2 + 1;
                        %%-----------------Get the output values of Moving Platform-----------------------
                        %%--------------------Calculate the position of Ai Bi Ci------------------
                        A2(jA2C2,:) = [0, L1/2, 0];
                        B2(jA2C2,:) = [- L2 * cos(q1q2(i,7)) * sin(q1q2(i,6)), L1/2 + L2 * cos(q1q2(i,7)) * cos(q1q2(i,6)), L2 * sin(q1q2(i,7))];
                        C2(jA2C2,:) = [- L2 * (cos(q1q2(i,7)) + cos(q1q2(i,7) + q1q2(i,8))) * sin(q1q2(i,6)), L1/2 + L2 * (cos(q1q2(i,7))...
                            + cos(q1q2(i,7) + q1q2(i,8))) * cos(q1q2(i,6)), L2 * (sin(q1q2(i,7)) + sin(q1q2(i,7) + q1q2(i,8)))];
                        %%------------------------------------------------------------------------
                        
                        if norm(C2(jA2C2,:) - C2_in_Ob) <= 1e-6
                            q2(jA2C2,1:5) = q1q2(i,6:10);
                            A2B2C(jA2C2,:) = [A2(jA2C2,:), B2(jA2C2,:), C2(jA2C2,:)];
                        end
                    end
                end
                
                % Here, I did a small trick:
                % The number of correct value of q1 and q2 might be different,
                % so, I force the number to be the same by compensating the
                % fewer one with the missing number (jA1C1-jA2C2) of first value q1(1,1:5) and A1B1C1(1,:)
                if jA1C1 ~= 0 && jA2C2 ~= 0 
                    WSvalue_1T2R = 1;
                    if jA1C1 > jA2C2
                        for i = 1:1:(jA1C1-jA2C2)
                            q2(jA2C2 + i,1:5) = q2(1,1:5);
                            A2B2C(jA2C2 + i,:) = A2B2C(1,:);
                        end
                    elseif jA1C1 < jA2C2
                        for i = 1:1:(jA2C2-jA1C1)
                            q1(jA1C1 + i,1:5) = q1(1,1:5);
                            A1B1C1(jA1C1 + i,:) = A1B1C1(1,:);
                        end
                    end
                        q1q2_FeasibleSolution = [q1(:,1:5), q2(:,1:5)];
                        ABC_FeasibleSolution = [A1B1C1(:,:),A2B2C(:,:)];
                else
                    WSvalue_1T2R = 0;
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                end                
                WSvalue_1T2R_SinguPosA1C1 = 0;
                WSvalue_1T2R_SinguPosA2C2 = 0;
            end
            WSvalue = [WSvalue_1T2R, WSvalue_1T2R_SinguPosA1C1, WSvalue_1T2R_SinguPosA2C2];      
            p = [po{1}, po{2}, po{3}, EulerAngle];     
        end
        
    end
end

