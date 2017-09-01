classdef RCBFixedSerialChain
    % RCBFixedSerialChain Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        l1;
        l2;
        pos;
        q11q12q21q22;
    end
    
    methods
        function obj = RCBFixedSerialChain(pos,q11q12q21q22 ,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q21q22 = q11q12q21q22 ;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_FixedSerialChain_IK(obj)
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
            
            q11 = po{7};
            q12 = po{8};
            q21 = po{9};
            q22 = po{10};
            
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
                name = 'FixedSerialChain';
                fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                    name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi,     po{7}*180/pi, po{8}*180/pi, po{9}*180/pi,po{10}*180/pi);
                alpha = 0;
                beta = 0;
                gamma = 0;
                theta = 0;
                EulerAngle = [alpha, beta, gamma];
                EulerAngle_q11_theta = [EulerAngle, q11, theta];
                
              %% ----------------------- Calculate one solutions for one input  -----------------------
                if p(1) == 0 && p(2) == 0 && p(3) == 0
                    q13 = pi;
                    q14 = pi/2 - (q12 + q13 + theta);
                    q15 = q11;
                    
                    q23 = pi;
                    q24 = pi/2 - (q22 + q23 + theta);
                    q25 = q21;
                    q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                end
                
                i = 1;
                jA1C1 = 0; % NumberofFeasibleSolutionA1C1 = 0;
                jA2C2 = 0; % NumberofFeasibleSolutionA2C2 = 0;
                %------------------Judge the workspace and solution existence of A1C1-------------------------
                %---------------------------Position of A1-C1 ----------------------------
                if q11 >= -2*pi && q12 >= -pi/4 && q13 >= -pi && q14 >= -135*pi/180 && q15 >= -2*pi...
                        && q11 <= 2*pi && q12 <= pi/4 && q13 <= pi && q14 <= 135*pi/180 && q15 <= 2*pi...
                        && isreal(q1q2(i,1:5)) ~= 0
                    jA1C1 = jA1C1 + 1;
                    %%-----------------Get the output values of Moving Platform-----------------------
                    %%--------------------Calculate the position of Ai Bi Ci------------------
                    A1(jA1C1,:) = [0, -L1/2, 0];
                    B1(jA1C1,:) = [L2 * cos(q1q2(i,2)) * sin(q1q2(i,1)), -L1/2 - L2 * cos(q1q2(i,2)) * cos(q1q2(i,1)), L2 * sin(q1q2(i,2))];
                    C1(jA1C1,:) = [L2 * (cos(q1q2(i,2)) + cos(q1q2(i,2) + q1q2(i,3))) * sin(q1q2(i,1)), -L1/2 - L2 * (cos(q1q2(i,2))...
                        + cos(q1q2(i,2) + q1q2(i,3))) * cos(q1q2(i,1)), L2 * (sin(q1q2(i,2)) + sin(q1q2(i,2) + q1q2(i,3)))];
                    %%------------------------------------------------------------------------
                    q1(jA1C1,1:5) = q1q2(i,1:5);
                    A1B1C1(jA1C1,:) = [A1(jA1C1,:), B1(jA1C1,:), C1(jA1C1,:)];
                end
                
                %%------------------------------------------------------------------------
                %---------------------------Position of A2-C2 ----------------------------
                if q21 >= -2*pi && q22 >= -pi/4 && q23 >= -pi && q24 >= -135*pi/180 && q25 >= -2*pi ...
                        && q21 <= 2*pi && q22 <= pi/4 && q23 <= pi && q24 <= 135*pi/180 && q25 <= 2*pi...
                        && isreal(q1q2(i,6:10)) ~= 0
                    jA2C2 = jA2C2 + 1;
                    %%-----------------Get the output values of Moving Platform-----------------------
                    %%--------------------Calculate the position of Ai Bi Ci------------------
                    A2(jA2C2,:) = [0, L1/2, 0];
                    B2(jA2C2,:) = [- L2 * cos(q1q2(i,7)) * sin(q1q2(i,6)), L1/2 + L2 * cos(q1q2(i,7)) * cos(q1q2(i,6)), L2 * sin(q1q2(i,7))];
                    C2(jA2C2,:) = [- L2 * (cos(q1q2(i,7)) + cos(q1q2(i,7) + q1q2(i,8))) * sin(q1q2(i,6)), L1/2 + L2 * (cos(q1q2(i,7))...
                        + cos(q1q2(i,7) + q1q2(i,8))) * cos(q1q2(i,6)), L2 * (sin(q1q2(i,7)) + sin(q1q2(i,7) + q1q2(i,8)))];
                    %%------------------------------------------------------------------------
                        q2(jA2C2,1:5) = q1q2(i,6:10);
                        A2B2C(jA2C2,:) = [A2(jA2C2,:), B2(jA2C2,:), C2(jA2C2,:)];
                end
            end
                % Here, I did a small trick:
                % The number of correct value of q1 and q2 might be different,
                % so, I force the number to be the same by compensating the
                % fewer one with the missing number (jA1C1-jA2C2) of first value q1(1,1:5) and A1B1C1(1,:)
                if jA1C1 ~= 0 && jA2C2 ~= 0
                    WSvalue_FixedSerialChain = 1;
                    %for i = 1:1:max(jA1C1,jA2C2)
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
                    WSvalue_FixedSerialChain = 0;
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                end                
                WSvalue_3T1R_SinguPosA1C1 = 0;
                WSvalue_3T1R_SinguPosA2C2 = 0;
                
                WSvalue = [WSvalue_FixedSerialChain, WSvalue_3T1R_SinguPosA1C1, WSvalue_3T1R_SinguPosA2C2];
                p = [po{1}, po{2}, po{3}, EulerAngle];
        end
        
        function [p, ABC, q1q2] = RCB_FixedSerialChain_FK(obj)
           
            q11 = obj.q11q12q21q22(1);
            q12 = obj.q11q12q21q22(2);
            q21 = obj.q11q12q21q22(3);
            q22 = obj.q11q12q21q22(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %% ----------------------- Calculate one solutions for one input  -----------------------
            
                q13 = pi;
                q14 = - pi/2 - q12;
                q15 = q11;
                
                q23 = pi;
                q24 = - pi/2 - q22;
                q25 = q21;
                q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                
               
               %%--------------------Calculate the position of Ai Bi Ci------------------
                A1 = [0, -L1/2, 0];
                B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
                C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
                
                A2 = [0, L1/2, 0];
                B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
                C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
                %%------------------------------------------------------------------------
            
                %----------------------Position of A1-C1 and A2-C2-------------------------
                ABC(:) = [A1, B1, C1, A2, B2, C2];
                p = [0 0 0, 0 0 0];
            
                
        end
        
    end
    
end

