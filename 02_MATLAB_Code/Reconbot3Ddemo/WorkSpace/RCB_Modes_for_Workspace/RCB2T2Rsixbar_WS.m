classdef RCB2T2Rsixbar_WS
    % RCB2T2Rsixbar operational mode
    % RCB3T1R is one mode with the motion of platfrom 3 Transition motion
    % and 1 rotation around z-axis        
    %%------------2-RER 3T1R inverse and Forward kinematics--------------
    %--------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with mode 2T2R in Six-bar case;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_3T1R_IK(obj)
    %     The FK function: [p, ABC, q1q2] = RCB_3T1R_FK(obj)    
    %     This is written by Wan Ding in 12 Dec 2016.
    %---------------------------------------------------------------------
    properties
        l1
        l2
        pos
        q11q12q14q23
    end
    
    methods
        function obj = RCB2T2Rsixbar_WS(pos,q11q12q14q23,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q14q23 = q11q12q14q23;             
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_2T2Rsixbar_IK(obj)
            % p = {-0.5, -0.5, 1, [], [], 0};
            % In the 2T2R mode, the IK can get the analytical solution by simplifing the ;
            %
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
            %-----------------------------------------------------------------------------
            
            L1 = obj.l1;
            L2 = obj.l2;
            po = obj.pos;   
            for i = 1:3
                if isempty(po{i}) == 0
                    p(i) = po{i};
                end
            end
            
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
            
            if isequal(p_BinaryCode, [1 1 1 0 0 0]) == 1
                % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
                % p = [0, 0, 0, [], [], []]
%                 display('Notice: Inputs are:p = [0, 0, 0, [], [], []]');
%                 display('Moving platform is fixed! p = [0, 0, 0, 0, 0, 0]');
                alpha = 0;
                beta = 0;
                gamma = 0;
                theta = 0;
                EulerAngle = [alpha, beta, gamma];
                EulerAngle_q11_theta = [EulerAngle, q11, theta];
                RotationMatrix = eul2rotm(EulerAngle);
                
                WSvalue_2T2R_SinguPosA1C1 = 1;
                WSvalue_2T2R_SinguPosA2C2 = 1;
            else
                
                % Euler angle IK
                % 1. Consider that when p(2)=0 (with two situation p(1)=0 or p(1)~=0), there exists no result of q11. how should we do?
                % 2. When calculates IK, which method should we use: Euler-angle or axis-angle?
                % -------Method I: Euler-angle-------- function [EulerAngle, ABC, q1q2] = RCB2T2RIKq11q12q14q23SixBar(p, EulerAngle_Input, l1, l2)
                %if isequal(p_BinaryCode, [1 1 1 0 0 1]) == 1
                    % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
                    % p = [x, y, z, [], [], gamma]
                    % display('Notice: Inputs are:p = [x, y, z, [], [], gamma]');
                    % display('Mechanism in a general six-bar linkage');               
                    if abs(theta) < 1e-8 || p(3) < 0
                        WSvalue = [0 0 0];
                        q1q2_FeasibleSolution = [];
                        ABC_FeasibleSolution = [];
                        EulerAngle_q11_theta = [];
                        return;
                    else
                        q11 = -atan(p(1)/p(2)); % inputs:  q11; atan(p(1)/p(2))>0(or<0), -(or +)q11 !!!!!
                    end  
                
                [EulerAngle_q11_theta] = EulerAngles_theta_q11_IK(theta, q11);
                if isempty(EulerAngle_q11_theta) == 1
                    WSvalue = [0 0 0];
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                    EulerAngle_q11_theta = [];
                    return;
                else           
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
                end
                po{4} = alpha;
                po{5} = beta;
                po{6} = gamma;
%                 name = '2T2R-SixBar';
%                     fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
%                         name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi);
            end
            
            %% -----------------------Calculate four possbile outputs for ABC(1:8), q1q2(1:8)-----------------------
            %--------------------- First step --------------------
            %Calculate k1(parallel to plane A1B1C1),k2(parallel to plane A2B2C2),
            % angles <A1C1,k1> and <A2C2,k2> ,lengths ||A1C1|| and ||A2C2||
            k1 = [-sin(q11), cos(q11), 0];
            k2 = [ sin(q21), -cos(q21), 0];
            A1C1 = C1_in_Ob - A1;
            A2C2 = C2_in_Ob - A2;
            if  norm(A1C1) <= 1e-12  % norm(A1C1) < 1e-6 
                angle_A1C1_k1 = pi/2;
                angle_A2C2_k2 = acos(dot(A2C2,k2)/(norm(A2C2)*norm(k2)));
            elseif  norm(A2C2) <= 1e-12 %|| norm(A2C2) < 1e-6
                angle_A2C2_k2 = pi/2;
                angle_A1C1_k1 = acos(dot(A1C1,k1)/(norm(A1C1)*norm(k1)));
            elseif norm(A1C1) <= 1e-12 && norm(A2C2) <= 1e-12
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
                q1 = [];
                q2 = [];
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
                    %q1q2(:,:) * 180 / pi
                    %------------------Judge the workspace and solution existence of A1C1-------------------------
                    if q11 > -pi/2 && q11 < pi/2
                        q14_LowerLimit = -120*pi/180;
                    else
                        q14_LowerLimit = -105*pi/180;
                    end
                    %---------------------------Position of A1-C1 ----------------------------
                    if q11 >= -2*pi && q12 >= 0 && q13 >= -pi && q14 >= q14_LowerLimit && q15 >= -2*pi...
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
                    if q21 == 0 || q21 == pi || q21 == -pi || q21 == 2*pi || q21 == -2*pi 
                        q22_LowerLimit = -pi/6;
                        q24_LowerLimit = -135*pi/180;
                    else                       
                        q22_LowerLimit = 0;
                        q24_LowerLimit = -105*pi/180;
                    end
                    
                    if q11 > -pi/2 && q11 < pi/2
                        q24_LowerLimit = -120*pi/180;
                    end 
                    
                    if q21 >= -2*pi && q22 >= q22_LowerLimit && q23 >= -pi && q24 >= q24_LowerLimit && q25 >= -2*pi ...
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
                if jA1C1 ~= 0 && jA2C2 ~= 0 && isempty(q1) ~= 1 && isempty(q2) ~= 1
                    WSvalue_2T2R = 1;
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
                    WSvalue_2T2R = 0;
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                end                
                WSvalue_2T2R_SinguPosA1C1 = 0;
                WSvalue_2T2R_SinguPosA2C2 = 0;
            
            WSvalue = [WSvalue_2T2R, WSvalue_2T2R_SinguPosA1C1, WSvalue_2T2R_SinguPosA2C2];      
            p = [po{1}, po{2}, po{3}, EulerAngle];           
        end
        
        function [p, ABC, q1q2] = RCB_2T2Rsixbar_FK(obj)
            % In the 2T2R mode, the FK can get the analytical solution by simplifing the ;
            %
            %%--------------2-RER 2T2R state forward kinematics-----------------
            %-------------------------------------------------------------------
            %     This function is to calculate the forward kinematics of 4 DoF 2-RER PM
            %   mechanism with mode 2T2R;
            %     The function: [p, ABC, q13q23] = RCB2T2RFK(q, l1, l2)
            %
            %   This is written by Wan Ding in 1 Okt 2016.
            %
            %       q                 input four angles q11 q12 q14 q22
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
            
            q11 = obj.q11q12q14q23(1);
            q12 = obj.q11q12q14q23(2);
            q14 = obj.q11q12q14q23(3);
            q23 = obj.q11q12q14q23(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %% -----------------------Basic calculaion of planar six-bar linkage-----------------------
            q21 = q11;
            l1pie = L1 * cos(q11);
            lA1A2pie = l1pie;
            lC1C2pie = l1pie;
            lA1B1pie = L2;
            lB1C1pie = L2;
            lB1C2pie = sqrt(l1pie^2 + L2^2 - 2 * l1pie * L2 * cos(pi - (pi/2+q14)));
            lB1A2pie = sqrt(l1pie^2 + L2^2 - 2 * l1pie * L2 * cos(pi - q12));
            lA2C2pie = 2 * L2 * cos(q23 / 2);
            
            % ------ Method I ---------
            angleA1A2B1pie = atan(L2 * sin(q12) / (l1pie + L2 * cos(q12)));
            
            if (pi/2+q14) < 0
                angleC1C2B1pie = - acos((l1pie^2 + lB1C2pie^2 - L2^2) / (2 * l1pie * lB1C2pie));
            else
                angleC1C2B1pie = acos((l1pie^2 + lB1C2pie^2 - L2^2) / (2 * l1pie * lB1C2pie));
            end
            %------ Method II ---------
            % angleA1A2B1pie = acos((lB1A2pie^2 + lA1A2pie^2 - lA1B1pie^2) / (2 * lB1A2pie * lA1A2pie));
            % angleA1B1A2pie = acos((lA1B1pie^2 + lB1A2pie^2 - lA1A2pie^2) / (2 * lA1B1pie * lB1A2pie));
            
            % angleC1C2B1pie = acos((lC1C2pie^2 + lB1C2pie^2 - lB1C1pie^2) / (2 * lC1C2pie * lB1C2pie));
            % angleC1B1C2pie = acos((lB1C2pie^2 + lB1C1pie^2 - lC1C2pie^2) / (2 * lB1C2pie * lB1C1pie));
            
            %---------Triangle A2B1C2 Inner three Angles----------
            angleC2B1A2pie = acos((lB1A2pie^2 + lB1C2pie^2 - lA2C2pie^2) / (2 * lB1A2pie * lB1C2pie));
            angleB1A2C2pie = acos((lB1A2pie^2 + lA2C2pie^2 - lB1C2pie^2) / (2 * lB1A2pie * lA2C2pie));
            angleA2C2B1pie = pi - angleC2B1A2pie - angleB1A2C2pie;
            
            %----------The output angle q13, q23, q24 can be calculated as follows:----
            q13 = pi - (q12 + (pi/2+q14) + angleC2B1A2pie - angleA1A2B1pie - angleC1C2B1pie);
            %     q13 = pi - (angleC2B1A2pie + angleA1B1A2pie + angleC1B1C2pie);
            q22 = pi - angleA1A2B1pie - angleB1A2C2pie - q23 / 2;
            q24 = - pi/2 + (pi - angleC1C2B1pie - angleA2C2B1pie - q23 / 2);
            %--------------------- output angle elimilate imagary part ------------------
            if isreal(q13)~= 1 && imag(q13) < 1e-6 || isreal(q22)~= 1 && imag(q22) < 1e-6 || isreal(q22)~= 1 && imag(q22) < 1e-6
                q13 = real(q13);
                q22 = real(q22);
                q24 = real(q24);
            end
            
            %% -----------------------Get the output values of Moving Platform-----------------------
            C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
            C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
            %norm(C1-C2)
            if norm(C1-C2) - L1 > 1e-6
                display('Notice:The solution is incorrect, mechanism recovery to original configuration')
                q1q2 = [0, pi/3, pi/3, pi/3, 0, 0, pi/3, pi/3, pi/3, 0];
            end
            %%--------------------Calculate the position of Ai Bi Ci------------------
            A1 = [0, -L1/2, 0];
            B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
            C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
            
            A2 = [0, L1/2, 0];
            B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
            C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
            %%------------------------------------------------------------------------
            
            %-------------------------q15 = q25------------------------------
            % Calculate the angles of q15
            q15 = q11;
            q25 = q21;
            %-------------------Transform into angle-------------------
            %q15_Angle = q15 * 180 / pi;
            
            %----------------------Position of A1-C1 and A2-C2-------------------------
            ABC = [A1, B1, C1, A2, B2, C2];
            %-------------------------q11-q15 and q21-q25------------------------------
            if norm(C1-C2) - L1 < 1e-6
                q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
            end
            %-------------Calculate the center point of moving platform----------------
            p = (C1 + C2) / 2;
            vectorC1C2 = C2 - C1;
            %%------------------------------------------------------------------------
            
            %% ----------------------Calculate the RPY angle--------------------------
            theta = pi/2 - q14 - (q12 + q13); % q24 = pi/2 - (q22 + q23 + theta);
            u_RotationAxis = [cos(q11), sin(q11), 0];
            r = [u_RotationAxis, theta];
            %-- m=vrrotvec2mat(r):Convert rotation from axis-angle to matrix representation--
            Matrix_from_axis_angle = vrrotvec2mat(r);
            %-- tform = rotm2tform(rotm): Convert rotation matrix to homogeneous transformation--
            Tform_from_axis_angle = rotm2tform(Matrix_from_axis_angle);
            %-- eul = tform2eul(tform): Extract Euler angles from homogeneous transformation--
            eul_alpha_beta_gamma = tform2eul(Tform_from_axis_angle,'ZYX');
            
            p(4:6) = eul_alpha_beta_gamma;
            %% --------------------Plot the mechanism Ai Bi Ci------------------
%                      PA1B1C1x = [A1(1), B1(1), C1(1)];
%                      PA1B1C1y = [A1(2), B1(2), C1(2)];
%                      PA1B1C1z = [A1(3), B1(3), C1(3)];
%                      plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
%             
%                      PA2B2C2x = [A2(1), B2(1), C2(1)];
%                      PA2B2C2y = [A2(2), B2(2), C2(2)];
%                      PA2B2C2z = [A2(3), B2(3), C2(3)];
%                      plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
%             
%                      PC1C2x = [C1(1), C2(1)];
%                      PC1C2y = [C1(2), C2(2)];
%                      PC1C2z = [C1(3), C2(3)];
%                      plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
%             
%                      PA1A2x = [A1(1), A2(1)];
%                      PA1A2y = [A1(2), A2(2)];
%                      PA1A2z = [A1(3), A2(3)];
%                      plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
%             
%                       %----------------- plot xyz axes of base point --------------
%                         x_axis = [50 0 0];
%                         y_axis = [0 50 0];
%                         z_axis = [0 0 50];
%                         OP= [0 0 0];
%                         xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
%                             i = 1:2;
%                             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
%                             i = 3:4;
%                             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
%                             i = 5:6;
%                             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
%                        %-----------------------------------------------------------
%                        %------------------plot xyz axes of Moving Platform----------------
%                         xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(Matrix_from_axis_angle * transpose(xyz));
%                         i = 1:2;
%                         plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
%                         hold on;
%                         axis equal;
%                         i = 3:4;
%                         plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
%                         i = 5:6;
%                         plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
%                        %----------------------------------------------
%             
%                      grid on;
%                      xlabel('x');
%                      ylabel('y');
%                      zlabel('z');
%                      axis equal;
            %%------------------------------------------------------------------------
        end
     
    end
    
end

