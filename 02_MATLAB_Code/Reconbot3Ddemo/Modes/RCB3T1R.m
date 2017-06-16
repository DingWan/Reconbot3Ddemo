classdef RCB3T1R
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
    
%      properties (Dependent)
%         RCB_3T1R_IK
%         RCB_3T1R_FK
%     end
        
    methods
        
        function obj = RCB3T1R(pos,q11q12q21q22,L1,L2)% constructor
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q21q22 = q11q12q21q22;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_3T1R_IK(obj)
            %%--------------2-RER 3T1R and 2T2R states inverse kinematics-----------------
            %-----------------------------------------------------------------------------
            %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
            %   mechanism with mode 2T2R in Six-bar case;
            %     The function: [EulerAngle_q11_theta, ABC, q1q2] = RCB3T1R_2T2RIK(p, l1, l2)
            %
            %   This is written by Wan Ding in 15 Nov 2016.
            %-----------------------------------------------------------------------------
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
            %       WSvalue_3T1R_SinguPosA1C1  Workspace value (0/1) in 3T1R mode as kinematic chain A1C1 singurlar
            %       WSvalue_3T1R_SinguPosA2C2  Workspace value (0/1) in 3T1R mode as kinematic chain A2C2 singurlar
            %       WSvalue_3T1R               Workspace value (0/1) in 3T1R mode as input 'po' is inside(1)/outside(0) the workspace
            %       WSvalue = [WSvalue_3T1R, WSvalue_3T1R_SinguPosA1C1, WSvalue_3T1R_SinguPosA2C2];
            %       jA1C1 = 0;                % NumberofFeasibleSolutionA1C1 = 0;    
            %       jA2C2 = 0;                % NumberofFeasibleSolutionA2C2 = 0;   
            
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
            
            switch length(po)
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
            %%
            for i = 1:6
                if isempty(po{i}) == 1
                    p_BinaryCode(i) = 0;
                else
                    p_BinaryCode(i) = 1;
                end
            end
            
            %% ----------------------- Calculate rotation matrix of 3T1R & 2T2R modes -----------------------
            
                % 3T1R mode:  [1 1 1 1 0 0]
                % p = [x, y, z, alpha, [], []];
                %name = '3T1R';                
                %fprintf('%s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', name, po{1}, po{2}, po{3}, po{4}, po{5}, po{6});
                RotationMatrix = rotz(alpha * 180 / pi);            
            
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
            
                % Judge the singularity as 1st+5th axes of C1A1 and C2A2 overlap
                % We assume that the precision is 0.02mm (1) as industry manipulators
                if abs(C1_in_Ob(1) - A1(1)) <= 1e-8 && abs(C1_in_Ob(2) - A1(2)) <= 1e-8 && (abs(C2_in_Ob(1) - A2(1)) > 1e-8 || abs(C2_in_Ob(2) - A2(2)) > 1e-8)
                    %display('Notice:The 1st+5th axes of kinematic chain A1C1 overlapped')
                    % Display the output value, we use angle to show it properly
                    name = '3T1R-A1C1 Singularity';
                    fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                        name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi, po{7}*180/pi);
                    %display('The solution can be decided by stratgies like (eg.2): ')
                    %display('1.Follow the angle of previous or next steps;')
                    %display('2.Transit into 2T2R, namely, q11 = -q21')
                    %----------------- Singularity Judgement --------------------
                    WSvalue_3T1R_SinguPosA1C1 = 1;
                    WSvalue_3T1R_SinguPosA2C2 = 0;
                    %--------------------- Assign Value --------------------
                    q21 = -atan((C2_in_Ob(1) - A2(1))/(C2_in_Ob(2) - A2(2))); % Clockwise is positive direction. top view
                    q11 = q11_SP_A1C1overlap;
                elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-8 && abs(C2_in_Ob(2) - A2(2)) <= 1e-8 && (abs(C1_in_Ob(1) - A1(1)) > 1e-8 || abs(C1_in_Ob(2) - A1(2)) > 1e-8)
                    %display('Notice:The 1st+5th axes of kinematic chain A2C2 overlapped')
                    name = '3T1R-A2C2 Singularity';
                    fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                        name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi, po{8}*180/pi);
                    %display('The solution can be decided by stratgies like(eg.2): ')
                    %display('1.Follow the angle of previous or next steps;')
                    %display('2.Transit into 2T2R, namely, q21 = -q11 ')
                    %----------------- Singularity Judgement --------------------
                    WSvalue_3T1R_SinguPosA1C1 = 0;
                    WSvalue_3T1R_SinguPosA2C2 = 1;
                    %--------------------- Assign Value --------------------
                    q11 = -atan((C1_in_Ob(1) - A1(1))/(C1_in_Ob(2) - A1(2))); % Anti-clockwise is positive direction. top view
                    q21 = q21_SP_A2C2overlap;
                    % Display the output value, we use angle to show it properly
                    
                elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-8 && abs(C1_in_Ob(2) - A1(2)) <= 1e-8 && abs(C2_in_Ob(1) - A2(1)) <= 1e-8 && abs(C2_in_Ob(2) - A2(2)) <= 1e-8
                    %display('Notice:The 1st+5th axes of kinematic chains A1C1 and A2C2 overlapped')
                    % Display the output value, we use angle to show it properly
                    name = '3T1R-A1C1+A2C2 Singularity';
                    fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                        name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi, po{7}*180/pi, po{8}*180/pi);
                    %display('The solution can be decided by stratgies like(eg.2): ')
                    %display('1.Follow the angle of previous or next steps;')
                    %display('2.Transit into 2T2R, namely, q11 = -q21 = 0 ')
                    %----------------- Singularity Judgement --------------------
                    WSvalue_3T1R_SinguPosA1C1 = 1;
                    WSvalue_3T1R_SinguPosA2C2 = 1;
                    %--------------------- Assign Value --------------------
                    q11 = q11_SP_A1C1_A2C2_overlap;
                    q21 = q21_SP_A1C1_A2C2_overlap;
                else
                    % Display the output value, we use angle to show it properly
                    name = '3T1R';
                    fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                        name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi);
                    %----------------- Singularity Judgement --------------------
                    WSvalue_3T1R_SinguPosA1C1 = 0;
                    WSvalue_3T1R_SinguPosA2C2 = 0;
                    %--------------------- Assign Value -------------------
                    q11 = -atan((C1_in_Ob(1) - A1(1))/(C1_in_Ob(2) - A1(2))); % Anti-clockwise is positive direction. top view
                    q21 = -atan((C2_in_Ob(1) - A2(1))/(C2_in_Ob(2) - A2(2))); % Anti-clockwise is positive direction. top view
                
                end
                theta = 0;
                beta = 0;
                gamma = 0;
                EulerAngle_q11_theta = [alpha, beta, gamma, q11, theta];
                EulerAngle = EulerAngle_q11_theta(1:3);
            
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
                q15 =  q11 - alpha;   q25 = q21 + alpha ;
                
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
                    
                    if C1(jA1C1,:) - C1_in_Ob <= 1e-8
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
                    
                    if C2(jA2C2,:) - C2_in_Ob <= 1e-8
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
                WSvalue_3T1R = 1;
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
                WSvalue_3T1R = 0;
                WSvalue_3T1R_SinguPosA1C1 = 0;
                WSvalue_3T1R_SinguPosA2C2 = 0;
                q1q2_FeasibleSolution = [];
                ABC_FeasibleSolution = [];
            end
            
            WSvalue = [WSvalue_3T1R, WSvalue_3T1R_SinguPosA1C1, WSvalue_3T1R_SinguPosA2C2];
            p = [po{1}, po{2}, po{3}, EulerAngle];
        end
        
        function [p, ABC, q1q2] = RCB_3T1R_FK(obj)
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
            F = cos(q11) * cos(q21) + sin(q11) * sin(q21);
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
            
            
            %%----------------------------SymbolicSolutiion3T1R_q11q12q21q22_Modified_C2x----------------------------
            x11 = sin(q11);
            y11 = cos(q11);
            x12 = sin(q12);
            y12 = cos(q12);
            x21 = sin(q21);
            y21 = cos(q21);
            x22 = sin(q22);
            y22 = cos(q22);           
            
            A1 = L2 * x11;
            A2 = L2 * x21;
            A3 = L2 * x11 * y12 + L2 * x21 * y22;
            A4 = L2 * y11;
            A5 = L2 * y21;
            A6 = L2 * y11 * y12 + L2 * y21 * y22 + L1;
            l1 = L1;
            l2 = L2;
            C1 =  (A1^4 - 4*A1^3*A3 + 2*A1^2*A2^2*x12^2 - 4*A1^2*A2^2*x12*x22 + 2*A1^2*A2^2*x22^2 - 2*A1^2*A2^2 + 6*A1^2*A3^2 + 2*A1^2*A4^2 - 4*A1^2*A4*A6 - 2*A1^2*A5^2*x12^2 + 4*A1^2*A5^2*x12*x22 - 2*A1^2*A5^2*x22^2 + 2*A1^2*A5^2 + 2*A1^2*A6^2 - 2*A1^2*l1^2 - 4*A1*A2^2*A3*x12^2 + 8*A1*A2^2*A3*x12*x22 - 4*A1*A2^2*A3*x22^2 + 4*A1*A2^2*A3 + 8*A1*A2*A4*A5*x12^2 - 16*A1*A2*A4*A5*x12*x22 + 8*A1*A2*A4*A5*x22^2 - 8*A1*A2*A4*A5 - 8*A1*A2*A5*A6*x12^2 + 16*A1*A2*A5*A6*x12*x22 - 8*A1*A2*A5*A6*x22^2 + 8*A1*A2*A5*A6 - 4*A1*A3^3 - 4*A1*A3*A4^2 + 8*A1*A3*A4*A6 + 4*A1*A3*A5^2*x12^2 - 8*A1*A3*A5^2*x12*x22 + 4*A1*A3*A5^2*x22^2 - 4*A1*A3*A5^2 - 4*A1*A3*A6^2 + 4*A1*A3*l1^2 + A2^4*x12^4 - 4*A2^4*x12^3*x22 + 6*A2^4*x12^2*x22^2 - 2*A2^4*x12^2 - 4*A2^4*x12*x22^3 + 4*A2^4*x12*x22 + A2^4*x22^4 - 2*A2^4*x22^2 + A2^4 + 2*A2^2*A3^2*x12^2 - 4*A2^2*A3^2*x12*x22 + 2*A2^2*A3^2*x22^2 - 2*A2^2*A3^2 - 2*A2^2*A4^2*x12^2 + 4*A2^2*A4^2*x12*x22 - 2*A2^2*A4^2*x22^2 + 2*A2^2*A4^2 + 4*A2^2*A4*A6*x12^2 - 8*A2^2*A4*A6*x12*x22 + 4*A2^2*A4*A6*x22^2 - 4*A2^2*A4*A6 + 2*A2^2*A5^2*x12^4 - 8*A2^2*A5^2*x12^3*x22 + 12*A2^2*A5^2*x12^2*x22^2 - 4*A2^2*A5^2*x12^2 - 8*A2^2*A5^2*x12*x22^3 + 8*A2^2*A5^2*x12*x22 + 2*A2^2*A5^2*x22^4 - 4*A2^2*A5^2*x22^2 + 2*A2^2*A5^2 - 2*A2^2*A6^2*x12^2 + 4*A2^2*A6^2*x12*x22 - 2*A2^2*A6^2*x22^2 + 2*A2^2*A6^2 + 2*A2^2*l1^2*x12^2 - 4*A2^2*l1^2*x12*x22 + 2*A2^2*l1^2*x22^2 - 2*A2^2*l1^2 - 8*A2*A3*A4*A5*x12^2 + 16*A2*A3*A4*A5*x12*x22 - 8*A2*A3*A4*A5*x22^2 + 8*A2*A3*A4*A5 + 8*A2*A3*A5*A6*x12^2 - 16*A2*A3*A5*A6*x12*x22 + 8*A2*A3*A5*A6*x22^2 - 8*A2*A3*A5*A6 + A3^4 + 2*A3^2*A4^2 - 4*A3^2*A4*A6 - 2*A3^2*A5^2*x12^2 + 4*A3^2*A5^2*x12*x22 - 2*A3^2*A5^2*x22^2 + 2*A3^2*A5^2 + 2*A3^2*A6^2 - 2*A3^2*l1^2 + A4^4 - 4*A4^3*A6 + 2*A4^2*A5^2*x12^2 - 4*A4^2*A5^2*x12*x22 + 2*A4^2*A5^2*x22^2 - 2*A4^2*A5^2 + 6*A4^2*A6^2 - 2*A4^2*l1^2 - 4*A4*A5^2*A6*x12^2 + 8*A4*A5^2*A6*x12*x22 - 4*A4*A5^2*A6*x22^2 + 4*A4*A5^2*A6 - 4*A4*A6^3 + 4*A4*A6*l1^2 + A5^4*x12^4 - 4*A5^4*x12^3*x22 + 6*A5^4*x12^2*x22^2 - 2*A5^4*x12^2 - 4*A5^4*x12*x22^3 + 4*A5^4*x12*x22 + A5^4*x22^4 - 2*A5^4*x22^2 + A5^4 + 2*A5^2*A6^2*x12^2 - 4*A5^2*A6^2*x12*x22 + 2*A5^2*A6^2*x22^2 - 2*A5^2*A6^2 + 2*A5^2*l1^2*x12^2 - 4*A5^2*l1^2*x12*x22 + 2*A5^2*l1^2*x22^2 - 2*A5^2*l1^2 + A6^4 - 2*A6^2*l1^2 + l1^4);
            C2 =  (8*A1^2*A2^2*x12 - 8*A1^2*A2^2*x22 - 8*A1^2*A5^2*x12 + 8*A1^2*A5^2*x22 - 16*A1*A2^2*A3*x12 + 16*A1*A2^2*A3*x22 + 32*A1*A2*A4*A5*x12 - 32*A1*A2*A4*A5*x22 - 32*A1*A2*A5*A6*x12 + 32*A1*A2*A5*A6*x22 + 16*A1*A3*A5^2*x12 - 16*A1*A3*A5^2*x22 + 8*A2^4*x12^3 - 24*A2^4*x12^2*x22 + 24*A2^4*x12*x22^2 - 8*A2^4*x12 - 8*A2^4*x22^3 + 8*A2^4*x22 + 8*A2^2*A3^2*x12 - 8*A2^2*A3^2*x22 - 8*A2^2*A4^2*x12 + 8*A2^2*A4^2*x22 + 16*A2^2*A4*A6*x12 - 16*A2^2*A4*A6*x22 + 16*A2^2*A5^2*x12^3 - 48*A2^2*A5^2*x12^2*x22 + 48*A2^2*A5^2*x12*x22^2 - 16*A2^2*A5^2*x12 - 16*A2^2*A5^2*x22^3 + 16*A2^2*A5^2*x22 - 8*A2^2*A6^2*x12 + 8*A2^2*A6^2*x22 + 8*A2^2*l1^2*x12 - 8*A2^2*l1^2*x22 - 32*A2*A3*A4*A5*x12 + 32*A2*A3*A4*A5*x22 + 32*A2*A3*A5*A6*x12 - 32*A2*A3*A5*A6*x22 - 8*A3^2*A5^2*x12 + 8*A3^2*A5^2*x22 + 8*A4^2*A5^2*x12 - 8*A4^2*A5^2*x22 - 16*A4*A5^2*A6*x12 + 16*A4*A5^2*A6*x22 + 8*A5^4*x12^3 - 24*A5^4*x12^2*x22 + 24*A5^4*x12*x22^2 - 8*A5^4*x12 - 8*A5^4*x22^3 + 8*A5^4*x22 + 8*A5^2*A6^2*x12 - 8*A5^2*A6^2*x22 + 8*A5^2*l1^2*x12 - 8*A5^2*l1^2*x22);
            C3 =  (8*A1^3*A3 - 4*A1^4 + 8*A1^2*A2^2 - 8*A1^2*A4^2 + 8*A1^2*A4*A6 - 8*A1^2*A5^2 - 8*A1*A2^2*A3*x12^2 + 16*A1*A2^2*A3*x12*x22 - 8*A1*A2^2*A3*x22^2 - 8*A1*A2^2*A3 + 32*A1*A2*A4*A5 - 16*A1*A2*A5*A6*x12^2 + 32*A1*A2*A5*A6*x12*x22 - 16*A1*A2*A5*A6*x22^2 - 16*A1*A2*A5*A6 - 8*A1*A3^3 + 8*A1*A3*A4^2 + 8*A1*A3*A5^2*x12^2 - 16*A1*A3*A5^2*x12*x22 + 8*A1*A3*A5^2*x22^2 + 8*A1*A3*A5^2 - 8*A1*A3*A6^2 + 8*A1*A3*l1^2 + 4*A2^4*x12^4 - 16*A2^4*x12^3*x22 + 24*A2^4*x12^2*x22^2 + 16*A2^4*x12^2 - 16*A2^4*x12*x22^3 - 32*A2^4*x12*x22 + 4*A2^4*x22^4 + 16*A2^4*x22^2 - 4*A2^4 + 8*A2^2*A3^2*x12^2 - 16*A2^2*A3^2*x12*x22 + 8*A2^2*A3^2*x22^2 - 8*A2^2*A4^2 + 8*A2^2*A4*A6*x12^2 - 16*A2^2*A4*A6*x12*x22 + 8*A2^2*A4*A6*x22^2 + 8*A2^2*A4*A6 + 8*A2^2*A5^2*x12^4 - 32*A2^2*A5^2*x12^3*x22 + 48*A2^2*A5^2*x12^2*x22^2 + 32*A2^2*A5^2*x12^2 - 32*A2^2*A5^2*x12*x22^3 - 64*A2^2*A5^2*x12*x22 + 8*A2^2*A5^2*x22^4 + 32*A2^2*A5^2*x22^2 - 8*A2^2*A5^2 - 8*A2^2*A6^2*x12^2 + 16*A2^2*A6^2*x12*x22 - 8*A2^2*A6^2*x22^2 + 8*A2^2*l1^2*x12^2 - 16*A2^2*l1^2*x12*x22 + 8*A2^2*l1^2*x22^2 - 16*A2*A3*A4*A5*x12^2 + 32*A2*A3*A4*A5*x12*x22 - 16*A2*A3*A4*A5*x22^2 - 16*A2*A3*A4*A5 + 32*A2*A3*A5*A6*x12^2 - 64*A2*A3*A5*A6*x12*x22 + 32*A2*A3*A5*A6*x22^2 + 4*A3^4 - 8*A3^2*A4*A6 - 8*A3^2*A5^2*x12^2 + 16*A3^2*A5^2*x12*x22 - 8*A3^2*A5^2*x22^2 + 8*A3^2*A6^2 - 8*A3^2*l1^2 - 4*A4^4 + 8*A4^3*A6 + 8*A4^2*A5^2 - 8*A4*A5^2*A6*x12^2 + 16*A4*A5^2*A6*x12*x22 - 8*A4*A5^2*A6*x22^2 - 8*A4*A5^2*A6 - 8*A4*A6^3 + 8*A4*A6*l1^2 + 4*A5^4*x12^4 - 16*A5^4*x12^3*x22 + 24*A5^4*x12^2*x22^2 + 16*A5^4*x12^2 - 16*A5^4*x12*x22^3 - 32*A5^4*x12*x22 + 4*A5^4*x22^4 + 16*A5^4*x22^2 - 4*A5^4 + 8*A5^2*A6^2*x12^2 - 16*A5^2*A6^2*x12*x22 + 8*A5^2*A6^2*x22^2 + 8*A5^2*l1^2*x12^2 - 16*A5^2*l1^2*x12*x22 + 8*A5^2*l1^2*x22^2 + 4*A6^4 - 8*A6^2*l1^2 + 4*l1^4);
            C4 =  (8*A1^2*A2^2*x22 - 8*A1^2*A2^2*x12 + 8*A1^2*A5^2*x12 - 8*A1^2*A5^2*x22 - 16*A1*A2^2*A3*x12 + 16*A1*A2^2*A3*x22 - 32*A1*A2*A4*A5*x12 + 32*A1*A2*A4*A5*x22 - 32*A1*A2*A5*A6*x12 + 32*A1*A2*A5*A6*x22 + 16*A1*A3*A5^2*x12 - 16*A1*A3*A5^2*x22 + 24*A2^4*x12^3 - 72*A2^4*x12^2*x22 + 72*A2^4*x12*x22^2 + 8*A2^4*x12 - 24*A2^4*x22^3 - 8*A2^4*x22 + 24*A2^2*A3^2*x12 - 24*A2^2*A3^2*x22 + 8*A2^2*A4^2*x12 - 8*A2^2*A4^2*x22 + 16*A2^2*A4*A6*x12 - 16*A2^2*A4*A6*x22 + 48*A2^2*A5^2*x12^3 - 144*A2^2*A5^2*x12^2*x22 + 144*A2^2*A5^2*x12*x22^2 + 16*A2^2*A5^2*x12 - 48*A2^2*A5^2*x22^3 - 16*A2^2*A5^2*x22 - 24*A2^2*A6^2*x12 + 24*A2^2*A6^2*x22 + 24*A2^2*l1^2*x12 - 24*A2^2*l1^2*x22 - 32*A2*A3*A4*A5*x12 + 32*A2*A3*A4*A5*x22 + 96*A2*A3*A5*A6*x12 - 96*A2*A3*A5*A6*x22 - 24*A3^2*A5^2*x12 + 24*A3^2*A5^2*x22 - 8*A4^2*A5^2*x12 + 8*A4^2*A5^2*x22 - 16*A4*A5^2*A6*x12 + 16*A4*A5^2*A6*x22 + 24*A5^4*x12^3 - 72*A5^4*x12^2*x22 + 72*A5^4*x12*x22^2 + 8*A5^4*x12 - 24*A5^4*x22^3 - 8*A5^4*x22 + 24*A5^2*A6^2*x12 - 24*A5^2*A6^2*x22 + 24*A5^2*l1^2*x12 - 24*A5^2*l1^2*x22);
            C5 =  (6*A1^4 - 4*A1^2*A2^2*x12^2 + 8*A1^2*A2^2*x12*x22 - 4*A1^2*A2^2*x22^2 - 12*A1^2*A2^2 - 12*A1^2*A3^2 + 12*A1^2*A4^2 + 4*A1^2*A5^2*x12^2 - 8*A1^2*A5^2*x12*x22 + 4*A1^2*A5^2*x22^2 + 12*A1^2*A5^2 - 4*A1^2*A6^2 + 4*A1^2*l1^2 - 16*A1*A2*A4*A5*x12^2 + 32*A1*A2*A4*A5*x12*x22 - 16*A1*A2*A4*A5*x22^2 - 48*A1*A2*A4*A5 - 16*A1*A3*A4*A6 + 6*A2^4*x12^4 - 24*A2^4*x12^3*x22 + 36*A2^4*x12^2*x22^2 + 36*A2^4*x12^2 - 24*A2^4*x12*x22^3 - 72*A2^4*x12*x22 + 6*A2^4*x22^4 + 36*A2^4*x22^2 + 6*A2^4 + 12*A2^2*A3^2*x12^2 - 24*A2^2*A3^2*x12*x22 + 12*A2^2*A3^2*x22^2 + 4*A2^2*A3^2 + 4*A2^2*A4^2*x12^2 - 8*A2^2*A4^2*x12*x22 + 4*A2^2*A4^2*x22^2 + 12*A2^2*A4^2 + 12*A2^2*A5^2*x12^4 - 48*A2^2*A5^2*x12^3*x22 + 72*A2^2*A5^2*x12^2*x22^2 + 72*A2^2*A5^2*x12^2 - 48*A2^2*A5^2*x12*x22^3 - 144*A2^2*A5^2*x12*x22 + 12*A2^2*A5^2*x22^4 + 72*A2^2*A5^2*x22^2 + 12*A2^2*A5^2 - 12*A2^2*A6^2*x12^2 + 24*A2^2*A6^2*x12*x22 - 12*A2^2*A6^2*x22^2 - 4*A2^2*A6^2 + 12*A2^2*l1^2*x12^2 - 24*A2^2*l1^2*x12*x22 + 12*A2^2*l1^2*x22^2 + 4*A2^2*l1^2 + 48*A2*A3*A5*A6*x12^2 - 96*A2*A3*A5*A6*x12*x22 + 48*A2*A3*A5*A6*x22^2 + 16*A2*A3*A5*A6 + 6*A3^4 - 4*A3^2*A4^2 - 12*A3^2*A5^2*x12^2 + 24*A3^2*A5^2*x12*x22 - 12*A3^2*A5^2*x22^2 - 4*A3^2*A5^2 + 12*A3^2*A6^2 - 12*A3^2*l1^2 + 6*A4^4 - 4*A4^2*A5^2*x12^2 + 8*A4^2*A5^2*x12*x22 - 4*A4^2*A5^2*x22^2 - 12*A4^2*A5^2 - 12*A4^2*A6^2 + 4*A4^2*l1^2 + 6*A5^4*x12^4 - 24*A5^4*x12^3*x22 + 36*A5^4*x12^2*x22^2 + 36*A5^4*x12^2 - 24*A5^4*x12*x22^3 - 72*A5^4*x12*x22 + 6*A5^4*x22^4 + 36*A5^4*x22^2 + 6*A5^4 + 12*A5^2*A6^2*x12^2 - 24*A5^2*A6^2*x12*x22 + 12*A5^2*A6^2*x22^2 + 4*A5^2*A6^2 + 12*A5^2*l1^2*x12^2 - 24*A5^2*l1^2*x12*x22 + 12*A5^2*l1^2*x22^2 + 4*A5^2*l1^2 + 6*A6^4 - 12*A6^2*l1^2 + 6*l1^4);
            C6 =  (8*A1^2*A2^2*x22 - 8*A1^2*A2^2*x12 + 8*A1^2*A5^2*x12 - 8*A1^2*A5^2*x22 + 16*A1*A2^2*A3*x12 - 16*A1*A2^2*A3*x22 - 32*A1*A2*A4*A5*x12 + 32*A1*A2*A4*A5*x22 + 32*A1*A2*A5*A6*x12 - 32*A1*A2*A5*A6*x22 - 16*A1*A3*A5^2*x12 + 16*A1*A3*A5^2*x22 + 24*A2^4*x12^3 - 72*A2^4*x12^2*x22 + 72*A2^4*x12*x22^2 + 8*A2^4*x12 - 24*A2^4*x22^3 - 8*A2^4*x22 + 24*A2^2*A3^2*x12 - 24*A2^2*A3^2*x22 + 8*A2^2*A4^2*x12 - 8*A2^2*A4^2*x22 - 16*A2^2*A4*A6*x12 + 16*A2^2*A4*A6*x22 + 48*A2^2*A5^2*x12^3 - 144*A2^2*A5^2*x12^2*x22 + 144*A2^2*A5^2*x12*x22^2 + 16*A2^2*A5^2*x12 - 48*A2^2*A5^2*x22^3 - 16*A2^2*A5^2*x22 - 24*A2^2*A6^2*x12 + 24*A2^2*A6^2*x22 + 24*A2^2*l1^2*x12 - 24*A2^2*l1^2*x22 + 32*A2*A3*A4*A5*x12 - 32*A2*A3*A4*A5*x22 + 96*A2*A3*A5*A6*x12 - 96*A2*A3*A5*A6*x22 - 24*A3^2*A5^2*x12 + 24*A3^2*A5^2*x22 - 8*A4^2*A5^2*x12 + 8*A4^2*A5^2*x22 + 16*A4*A5^2*A6*x12 - 16*A4*A5^2*A6*x22 + 24*A5^4*x12^3 - 72*A5^4*x12^2*x22 + 72*A5^4*x12*x22^2 + 8*A5^4*x12 - 24*A5^4*x22^3 - 8*A5^4*x22 + 24*A5^2*A6^2*x12 - 24*A5^2*A6^2*x22 + 24*A5^2*l1^2*x12 - 24*A5^2*l1^2*x22);
            C7 =  (8*A1^2*A2^2 - 8*A1^3*A3 - 4*A1^4 - 8*A1^2*A4^2 - 8*A1^2*A4*A6 - 8*A1^2*A5^2 + 8*A1*A2^2*A3*x12^2 - 16*A1*A2^2*A3*x12*x22 + 8*A1*A2^2*A3*x22^2 + 8*A1*A2^2*A3 + 32*A1*A2*A4*A5 + 16*A1*A2*A5*A6*x12^2 - 32*A1*A2*A5*A6*x12*x22 + 16*A1*A2*A5*A6*x22^2 + 16*A1*A2*A5*A6 + 8*A1*A3^3 - 8*A1*A3*A4^2 - 8*A1*A3*A5^2*x12^2 + 16*A1*A3*A5^2*x12*x22 - 8*A1*A3*A5^2*x22^2 - 8*A1*A3*A5^2 + 8*A1*A3*A6^2 - 8*A1*A3*l1^2 + 4*A2^4*x12^4 - 16*A2^4*x12^3*x22 + 24*A2^4*x12^2*x22^2 + 16*A2^4*x12^2 - 16*A2^4*x12*x22^3 - 32*A2^4*x12*x22 + 4*A2^4*x22^4 + 16*A2^4*x22^2 - 4*A2^4 + 8*A2^2*A3^2*x12^2 - 16*A2^2*A3^2*x12*x22 + 8*A2^2*A3^2*x22^2 - 8*A2^2*A4^2 - 8*A2^2*A4*A6*x12^2 + 16*A2^2*A4*A6*x12*x22 - 8*A2^2*A4*A6*x22^2 - 8*A2^2*A4*A6 + 8*A2^2*A5^2*x12^4 - 32*A2^2*A5^2*x12^3*x22 + 48*A2^2*A5^2*x12^2*x22^2 + 32*A2^2*A5^2*x12^2 - 32*A2^2*A5^2*x12*x22^3 - 64*A2^2*A5^2*x12*x22 + 8*A2^2*A5^2*x22^4 + 32*A2^2*A5^2*x22^2 - 8*A2^2*A5^2 - 8*A2^2*A6^2*x12^2 + 16*A2^2*A6^2*x12*x22 - 8*A2^2*A6^2*x22^2 + 8*A2^2*l1^2*x12^2 - 16*A2^2*l1^2*x12*x22 + 8*A2^2*l1^2*x22^2 + 16*A2*A3*A4*A5*x12^2 - 32*A2*A3*A4*A5*x12*x22 + 16*A2*A3*A4*A5*x22^2 + 16*A2*A3*A4*A5 + 32*A2*A3*A5*A6*x12^2 - 64*A2*A3*A5*A6*x12*x22 + 32*A2*A3*A5*A6*x22^2 + 4*A3^4 + 8*A3^2*A4*A6 - 8*A3^2*A5^2*x12^2 + 16*A3^2*A5^2*x12*x22 - 8*A3^2*A5^2*x22^2 + 8*A3^2*A6^2 - 8*A3^2*l1^2 - 4*A4^4 - 8*A4^3*A6 + 8*A4^2*A5^2 + 8*A4*A5^2*A6*x12^2 - 16*A4*A5^2*A6*x12*x22 + 8*A4*A5^2*A6*x22^2 + 8*A4*A5^2*A6 + 8*A4*A6^3 - 8*A4*A6*l1^2 + 4*A5^4*x12^4 - 16*A5^4*x12^3*x22 + 24*A5^4*x12^2*x22^2 + 16*A5^4*x12^2 - 16*A5^4*x12*x22^3 - 32*A5^4*x12*x22 + 4*A5^4*x22^4 + 16*A5^4*x22^2 - 4*A5^4 + 8*A5^2*A6^2*x12^2 - 16*A5^2*A6^2*x12*x22 + 8*A5^2*A6^2*x22^2 + 8*A5^2*l1^2*x12^2 - 16*A5^2*l1^2*x12*x22 + 8*A5^2*l1^2*x22^2 + 4*A6^4 - 8*A6^2*l1^2 + 4*l1^4);
            C8 =  (8*A1^2*A2^2*x12 - 8*A1^2*A2^2*x22 - 8*A1^2*A5^2*x12 + 8*A1^2*A5^2*x22 + 16*A1*A2^2*A3*x12 - 16*A1*A2^2*A3*x22 + 32*A1*A2*A4*A5*x12 - 32*A1*A2*A4*A5*x22 + 32*A1*A2*A5*A6*x12 - 32*A1*A2*A5*A6*x22 - 16*A1*A3*A5^2*x12 + 16*A1*A3*A5^2*x22 + 8*A2^4*x12^3 - 24*A2^4*x12^2*x22 + 24*A2^4*x12*x22^2 - 8*A2^4*x12 - 8*A2^4*x22^3 + 8*A2^4*x22 + 8*A2^2*A3^2*x12 - 8*A2^2*A3^2*x22 - 8*A2^2*A4^2*x12 + 8*A2^2*A4^2*x22 - 16*A2^2*A4*A6*x12 + 16*A2^2*A4*A6*x22 + 16*A2^2*A5^2*x12^3 - 48*A2^2*A5^2*x12^2*x22 + 48*A2^2*A5^2*x12*x22^2 - 16*A2^2*A5^2*x12 - 16*A2^2*A5^2*x22^3 + 16*A2^2*A5^2*x22 - 8*A2^2*A6^2*x12 + 8*A2^2*A6^2*x22 + 8*A2^2*l1^2*x12 - 8*A2^2*l1^2*x22 + 32*A2*A3*A4*A5*x12 - 32*A2*A3*A4*A5*x22 + 32*A2*A3*A5*A6*x12 - 32*A2*A3*A5*A6*x22 - 8*A3^2*A5^2*x12 + 8*A3^2*A5^2*x22 + 8*A4^2*A5^2*x12 - 8*A4^2*A5^2*x22 + 16*A4*A5^2*A6*x12 - 16*A4*A5^2*A6*x22 + 8*A5^4*x12^3 - 24*A5^4*x12^2*x22 + 24*A5^4*x12*x22^2 - 8*A5^4*x12 - 8*A5^4*x22^3 + 8*A5^4*x22 + 8*A5^2*A6^2*x12 - 8*A5^2*A6^2*x22 + 8*A5^2*l1^2*x12 - 8*A5^2*l1^2*x22);
            C9 =  (A1^4 + 4*A1^3*A3 + 2*A1^2*A2^2*x12^2 - 4*A1^2*A2^2*x12*x22 + 2*A1^2*A2^2*x22^2 - 2*A1^2*A2^2 + 6*A1^2*A3^2 + 2*A1^2*A4^2 + 4*A1^2*A4*A6 - 2*A1^2*A5^2*x12^2 + 4*A1^2*A5^2*x12*x22 - 2*A1^2*A5^2*x22^2 + 2*A1^2*A5^2 + 2*A1^2*A6^2 - 2*A1^2*l1^2 + 4*A1*A2^2*A3*x12^2 - 8*A1*A2^2*A3*x12*x22 + 4*A1*A2^2*A3*x22^2 - 4*A1*A2^2*A3 + 8*A1*A2*A4*A5*x12^2 - 16*A1*A2*A4*A5*x12*x22 + 8*A1*A2*A4*A5*x22^2 - 8*A1*A2*A4*A5 + 8*A1*A2*A5*A6*x12^2 - 16*A1*A2*A5*A6*x12*x22 + 8*A1*A2*A5*A6*x22^2 - 8*A1*A2*A5*A6 + 4*A1*A3^3 + 4*A1*A3*A4^2 + 8*A1*A3*A4*A6 - 4*A1*A3*A5^2*x12^2 + 8*A1*A3*A5^2*x12*x22 - 4*A1*A3*A5^2*x22^2 + 4*A1*A3*A5^2 + 4*A1*A3*A6^2 - 4*A1*A3*l1^2 + A2^4*x12^4 - 4*A2^4*x12^3*x22 + 6*A2^4*x12^2*x22^2 - 2*A2^4*x12^2 - 4*A2^4*x12*x22^3 + 4*A2^4*x12*x22 + A2^4*x22^4 - 2*A2^4*x22^2 + A2^4 + 2*A2^2*A3^2*x12^2 - 4*A2^2*A3^2*x12*x22 + 2*A2^2*A3^2*x22^2 - 2*A2^2*A3^2 - 2*A2^2*A4^2*x12^2 + 4*A2^2*A4^2*x12*x22 - 2*A2^2*A4^2*x22^2 + 2*A2^2*A4^2 - 4*A2^2*A4*A6*x12^2 + 8*A2^2*A4*A6*x12*x22 - 4*A2^2*A4*A6*x22^2 + 4*A2^2*A4*A6 + 2*A2^2*A5^2*x12^4 - 8*A2^2*A5^2*x12^3*x22 + 12*A2^2*A5^2*x12^2*x22^2 - 4*A2^2*A5^2*x12^2 - 8*A2^2*A5^2*x12*x22^3 + 8*A2^2*A5^2*x12*x22 + 2*A2^2*A5^2*x22^4 - 4*A2^2*A5^2*x22^2 + 2*A2^2*A5^2 - 2*A2^2*A6^2*x12^2 + 4*A2^2*A6^2*x12*x22 - 2*A2^2*A6^2*x22^2 + 2*A2^2*A6^2 + 2*A2^2*l1^2*x12^2 - 4*A2^2*l1^2*x12*x22 + 2*A2^2*l1^2*x22^2 - 2*A2^2*l1^2 + 8*A2*A3*A4*A5*x12^2 - 16*A2*A3*A4*A5*x12*x22 + 8*A2*A3*A4*A5*x22^2 - 8*A2*A3*A4*A5 + 8*A2*A3*A5*A6*x12^2 - 16*A2*A3*A5*A6*x12*x22 + 8*A2*A3*A5*A6*x22^2 - 8*A2*A3*A5*A6 + A3^4 + 2*A3^2*A4^2 + 4*A3^2*A4*A6 - 2*A3^2*A5^2*x12^2 + 4*A3^2*A5^2*x12*x22 - 2*A3^2*A5^2*x22^2 + 2*A3^2*A5^2 + 2*A3^2*A6^2 - 2*A3^2*l1^2 + A4^4 + 4*A4^3*A6 + 2*A4^2*A5^2*x12^2 - 4*A4^2*A5^2*x12*x22 + 2*A4^2*A5^2*x22^2 - 2*A4^2*A5^2 + 6*A4^2*A6^2 - 2*A4^2*l1^2 + 4*A4*A5^2*A6*x12^2 - 8*A4*A5^2*A6*x12*x22 + 4*A4*A5^2*A6*x22^2 - 4*A4*A5^2*A6 + 4*A4*A6^3 - 4*A4*A6*l1^2 + A5^4*x12^4 - 4*A5^4*x12^3*x22 + 6*A5^4*x12^2*x22^2 - 2*A5^4*x12^2 - 4*A5^4*x12*x22^3 + 4*A5^4*x12*x22 + A5^4*x22^4 - 2*A5^4*x22^2 + A5^4 + 2*A5^2*A6^2*x12^2 - 4*A5^2*A6^2*x12*x22 + 2*A5^2*A6^2*x22^2 - 2*A5^2*A6^2 + 2*A5^2*l1^2*x12^2 - 4*A5^2*l1^2*x12*x22 + 2*A5^2*l1^2*x22^2 - 2*A5^2*l1^2 + A6^4 - 2*A6^2*l1^2 + l1^4);
            % the coefficient matrix of the 8-degree polynomials equation
            CoefficientMatrix8_Sym = [C1, C2, C3, C4, C5, C6, C7, C8, C9];
            % calculate the root of the 8-degree polynomials equation
            x_Sym = roots(CoefficientMatrix8_Sym);
            
            % the final value of "+/-theta13"
            % the programm should judge the sign of the value            
            j =0;
            for i = 1:length (x)
                if  abs( abs(2 * atan(x(i)) - q12) - pi ) > 1e-8
                    j = j + 1;
                    q13all(j) =  2 * atan(x(i))  - q12;
                else
                    continue;
                end
            end
            %% ------------------Obtain all of the correct values and assign to q13q23-------------------------
            % numbers of i and j are used to count the possible values that satisfy the
            % condiation of C1z = C2z; And then, assign all the possible values to
            % matrix q13q23;
            
            %tic
            for Numq13 = 1:length(q13all)
                
                q13SingleValue = q13all(Numq13);
                
              %% -------------- Calculate the value of q23 (theta23) ----------------
                % (q22+q23all)*180/pi
                % sin(q12) + sin(q12+q13SingleValue) - sin(q22) - sin(q22+q23all) == 0
                %  For real elements of x in the interval [-1,1], asin(x) returns values in the interval [-pi/2,pi/2]
                % iterative method to get the optimal value
                % 
                j = 0;
                delta_q13 = 0.1;
                while(abs(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) >= 1 && j <= 500)
                    j = j + 1;
                    if q12 + q13SingleValue > pi/2 || (q12 + q13SingleValue > -pi/2 && q12 + q13SingleValue <= 0)
                        q13SingleValue = q13SingleValue + delta_q13;
                    elseif q12 + q13SingleValue < -pi/2 || (q12 + q13SingleValue > 0 && q12 + q13SingleValue < pi/2)
                        q13SingleValue = q13SingleValue - delta_q13;
                    end
                end
                %
                q23all(1) = asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
                if sin(q12) + sin(q12 + q13SingleValue) - sin(q22) <= 0
                    q23all(2) =  - pi - asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
                else
                    q23all(2) =    pi - asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
                end
                
                for Numq23 = 1:length(q23all)
                    q23SingleValue = q23all(Numq23);
                    %%-----------------------Get the output values of Moving Platform-----------------------
                    %%--------------------Calculate the position of Ai Bi Ci------------------
                    A1 = [0, -L1/2, 0];
                    B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                    
                    A2 = [0, L1/2, 0];
                    B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
                    C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                        L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                    %%------------------------------------------------------------------------
                    JudgeLength_C1C2(Numq23) = norm(C1 - C2) - L1;
                    % Plot;
                end
                
                %Choose the column of minmum solution
                [~,col] = find(JudgeLength_C1C2 == min(JudgeLength_C1C2));
                
                %% Note:::::::::::::
                %  The value from 8-degree polynomials equation is not correct!!!!!
                % In order to get the right value, here, I use the calculated minimum q13SingleValue data to approch the correct value
                
                % Approching Algorithm is a iterative pocesss see below in
                % While loop
                if  abs(C1(3) - C2(3)) > 1e-8 || abs(abs( q13all(Numq13) ) - pi) > 1e-8 || abs(JudgeLength_C1C2(col(1))) > 1e-8
                    
                    %--------------------------------------------------------------------
                    if abs(C1(3) - C2(3)) < 1e-8 && abs(abs( q13all(Numq13) ) - pi) > 1e-8 && ...
                            abs(abs( q23all(col(1)) ) - pi) > 1e-8 && abs(JudgeLength_C1C2(col(1))) < 1e-8
                        JudgeLength_C1C2_min = JudgeLength_C1C2(col(1));
                    else
                        %---------- Method I ----------
                        for OnlyUsedForFolding_CircleOptimalPick = 1:1
                            % We simple the one cycle points
                            ii = 0;
                            for i_angle = 0:1*pi/180:pi
                                q13SingleValue = i_angle;
                                %
                                sin_value = sin(q12) + sin(q12 + q13SingleValue) - sin(q22);
                                zC1 = sin(q12) + sin(q12 + q13SingleValue);
                                %
                                if sin_value > 1 || zC1 < 0
                                    continue;
                                end
                                ii = ii + 1;
                                q13SingleValue_mat(ii) = i_angle;
                                %
                                q23all(1) = asin(sin_value) - q22;
                                if sin_value <= 0
                                    q23all(2) =  - pi - asin(sin_value) - q22;
                                else
                                    q23all(2) =    pi - asin(sin_value) - q22;
                                end
                                for i_q23all = 1:2
                                    q23SingleValue = q23all(i_q23all);
                                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                    C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                                        L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                                    JudgeLength_C1C2_Solution(i_q23all,ii) = norm(C1 - C2) - L1;
                                    q23SingleValue_mat(i_q23all,ii) = q23all(i_q23all);
                                    if i_q23all == 1
                                        C11_mat(ii,:) = C1;
                                        C21_mat(ii,:) = C2;
                                    else
                                        C12_mat(ii,:) = C1;
                                        C22_mat(ii,:) = C2;
                                    end
                                end
                                
                            end
                            plot3(C11_mat(:,1), C11_mat(:,2), C11_mat(:,3),'k-','linewidth',1); hold on;
                            plot3(C21_mat(:,1), C21_mat(:,2), C21_mat(:,3),'k-','linewidth',1); hold on;
                            plot3(C12_mat(:,1), C12_mat(:,2), C12_mat(:,3),'k-.','linewidth',1); hold on;
                            plot3(C22_mat(:,1), C22_mat(:,2), C22_mat(:,3),'k-.','linewidth',1); hold on;
                            
                            JudgeLength_C1C2_min = [];
                            for i_q23all = 1:2
                                B = JudgeLength_C1C2_Solution(i_q23all,:);
                                col_B_positive = find(B>0);
                                col_B_zero = find(abs(B) <= 1e-8);
                                if isempty(col_B_zero) ~= 1 && min(col_B_zero) < length(B)
                                    Changepoint_positive_min = min(col_B_zero);
                                    i_q23all_selected = i_q23all;
                                    JudgeLength_C1C2_min = 0;
                                elseif isempty(col_B_zero) ~= 1 && min(col_B_zero) == length(B)
                                    continue
                                else
                                    j = 0;
                                    Changepoint_postive = [];
                                    for i = 1:length(col_B_positive)
                                        if i == 1 && col_B_positive(1)>1
                                            j = j + 1;
                                            Changepoint_postive(j) = col_B_positive(i);
                                        elseif i < length(col_B_positive) && col_B_positive(i+1) - col_B_positive(i) > 1
                                            j = j + 1;
                                            Changepoint_postive(j) = col_B_positive(i);
                                            j = j + 1;
                                            Changepoint_postive(j) = col_B_positive(i+1);
                                        elseif i == length(col_B_positive) && col_B_positive(length(col_B_positive))<length(B)
                                            j = j + 1;
                                            Changepoint_postive(j) = col_B_positive(i);
                                        end
                                        Changepoint_positive_min = min(Changepoint_postive);
                                    end
                                    
                                    if isempty(Changepoint_postive) == 1
                                        if isempty(JudgeLength_C1C2_min) == 1
                                            JudgeLength_C1C2_min = [];
                                            Changepoint_positive_min = [];
                                        end
                                        continue
                                    else
                                        JudgeLength_C1C2_min = min(B(Changepoint_postive));
                                        i_q23all_selected = i_q23all;
                                    end
                                end
                            end  
                                q13SingleValue = q13SingleValue_mat(Changepoint_positive_min);
                                q23SingleValue = q23SingleValue_mat(i_q23all_selected,Changepoint_positive_min);
                                C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                    L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                                    L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                                
                                plot3(C12_mat(Changepoint_positive_min,1), C12_mat(Changepoint_positive_min,2), C12_mat(Changepoint_positive_min,3),'r.','markersize',10); hold on;
                                
                                %---------------------- Iterative Start --------------------
                                delta_q13 = 0.05;
                                q13SingleValue = q13SingleValue - delta_q13;
                                % norm(C1 - C2) - L1
                                SignChange_1 = 0;
                                SignChange_2 = 0;
                                k = 0;
                                % iterative method to get the optimal value
                                while(abs(norm(C1 - C2) - L1) > 1e-8 && k <= 500)
                                    k = k + 1;
                                    % Here, we must make sure 'sin_value belongs to [-1 1]'
                                    sin_value = sin(q12) + sin(q12 + q13SingleValue) - sin(q22);
                                    %
                                    q23all(1) = asin(sin_value) - q22;
                                    if sin_value <= 0
                                        q23all(2) =  - pi - asin(sin_value) - q22;
                                    else
                                        q23all(2) =    pi - asin(sin_value) - q22;
                                    end
                                    
                                    % Choose the correct q23all
                                    q23SingleValue = q23all(i_q23all_selected);
                                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                    C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                                        L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                                    
                                    JudgeLength_C1C2_min = norm(C1 - C2) - L1;
                                    %JudgeLength_C1C2_min
                                    if JudgeLength_C1C2_min > 0
                                        if SignChange_1 == 0
                                            if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min < 0
                                                delta_q13 = 0.5 * delta_q13;
                                            elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min >= 0
                                                delta_q13 = - 0.5 * delta_q13;
                                            end
                                        end
                                        SignChange_1 = SignChange_1 + 1;
                                        SignChange_2 = 0;
                                    elseif JudgeLength_C1C2_min < 0
                                        if SignChange_2 == 0
                                            if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min > 0
                                                delta_q13 = 0.5 * delta_q13;
                                            elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min <= 0
                                                delta_q13 = - 0.5 * delta_q13;
                                            end
                                        end
                                        SignChange_1 = 0;
                                        SignChange_2 = SignChange_2 + 1;
                                    end
                                    
                                    q13SingleValue = q13SingleValue - delta_q13;
                                    %hold off
                                    %Plot;
                                end
                                
                                %---------------------- Iterative end --------------------
                        end
                    end
                    %--------------------------------------------------------------------
                    
                    % assign the values to q13q23
                    q13 = q13SingleValue;
                    q23 = q23SingleValue;
                    
                    % Calculate the center point of moving platform
                    p(1:3) = (C1 + C2) / 2;
                    C1C2 = C2 - C1;
                    yaxis = [0, 1, 0];
                    % Judge the direction of the rotation around Z-axis
                    if C1(1) < C2(1)
                        gamma = -acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
                    else
                        gamma = acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
                    end
                    p(4:6) = [0, 0, gamma];
                    ABC = [A1, B1, C1, A2, B2, C2];
                    
                    %%-------------------------q14q15 and q24q25------------------------------
                    % Calculate the angles of q14q15 and q24q25
                    q14 = pi/2 - (q12 + q13);
                    q15 = q11 - gamma;
                    q24 = pi/2 - (q22 + q23);
                    q25 = q21 + gamma;
                    %%------------------------------------------------------------------------
                    %%-------------------------q11-q15 and q21-q25------------------------------
                    q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
                    if isempty(JudgeLength_C1C2_min) == 1 || abs(JudgeLength_C1C2_min) < 1e-8
                        if isempty(JudgeLength_C1C2_min) == 1 || abs( abs(q13) - pi) < 1e-8
                            display('Only Zero Position exist, Calculation process is stopped!');
                        end
                        break
                    else
                        continue
                    end
                elseif abs( abs(q13all(Numq13)) - pi ) < 1e-8 && abs( JudgeLength_C1C2(col(1)) ) < 1e-8
                    display('Zero Position, Calculation process needs to check!');
                    p = [];
                    ABC = [];
                    q1q2 = [0, pi/3, pi/3, -pi/6, 0, 0, pi/3, pi/3, -pi/6, 0];
                else
                    display('No solution for this input, Calculation process is stopped');
                    p = [];
                    ABC = [];
                    q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
                end
                
            end
            %toc
            
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
            
            grid on;
            axis equal;
            xlabel('x');
            ylabel('y');
            zlabel('z');
            
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
            axis equal;
            %-----------------------------------------------------------
            
        end
               
    end
    
end

function [c, ceq] = Z_value_equal(x, q12, q22) 

ceq = sin(q12) + sin(q12 + x(1)) - sin(q22) - sin(q22 + x(2));
c = [];
end