classdef RCB3T1RSingularityA1C1A2C2
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
        q11q12q21q23;
    end
            
    methods
        
        function obj = RCB3T1RSingularityA1C1A2C2(pos,q11q12q21q23,L1,L2)% constructor
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q21q23 = q11q12q21q23;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_3T1R_SingularityA1C1A2C2_IK(obj)
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
            k1 = [-sin(q11), cos(q11), 0];
            k2 = [-sin(q21), cos(q21), 0];
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
                    q24 = pi/2 - (q22 + q23 + theta);
                else
                    q22 = angle_A2C2_k2 - angleB2A2C2;
                    q24 = pi/2 + theta - (q22 + q23);
                end
                %--------- q15 and q25 -----------%
                q15 = q11 - alpha;   q25 = q21 + alpha ;
                
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
                    B2(jA2C2,:) = [L2 * cos(q1q2(i,7)) * sin(q1q2(i,6)), L1/2 - L2 * cos(q1q2(i,7)) * cos(q1q2(i,6)), L2 * sin(q1q2(i,7))];
                    C2(jA2C2,:) = [L2 * (cos(q1q2(i,7)) + cos(q1q2(i,7) + q1q2(i,8))) * sin(q1q2(i,6)), L1/2 - L2 * (cos(q1q2(i,7))...
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
            if jA1C1 ~= 0 && jA2C2 ~= 0 
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
        
        function [p, ABC, q1q2] = RCB_3T1R_SingularityA1C1A2C2_FK(obj)
            %[p, ABC, q1q2] = RCB3T1RFKq11q12q21q23(q, l1, l2)
            %%-----------Inputs-------------
            % In the 3T1R mode, the FK can't get the analytical solution, so, we must solve
            % the 8-degree polynomials equations;
            %
            % Here, I add two methods to solve FK:
            % Method I : Numerical Method (8-degree Polynomials)
            % Method II: Iterative Method (Set q11,q12,q23, then, sample q12 = 0~angle(C2z>0), obtain q13, find ||C1-C2||-L1 < 1e-10 )
            % Note: Method II activates as Method I can't get right value
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
            %       q                 input four angles q11 q12 q21 q23
            %       q13all            all possible angles of q13 that satisfy C1z = C2z
            %       q22all            all possible angles of q22 that satisfy C1z = C2z
            %       q13singlevalue    one of all possible angles of q13all
            %       q22singlevalue    one of all possible angles of q22all
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
            %tic
            % Variable value assignment
            q11 = obj.q11q12q21q23(1);
            q12 = obj.q11q12q21q23(2);
            q21 = obj.q11q12q21q23(3);
            q23 = obj.q11q12q21q23(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %%------------------------------ Method I --------------------------------
            %%--------------- Calculate the value of q22 (theta22) -------------------
                                    
            %%--------------------------------------- Method II ----------------------------------------------
            %%----------------------SymbolicSolutiion3T1R_q11q12q21q22_Modify_C2x_C2y-------------------------
            l1 = L1;
            l2 = L2;
            
            x11 = sin(q11);
            y11 = cos(q11);
            x12 = sin(q12);
            y12 = cos(q12);
            x21 = sin(q21);
            y21 = cos(q21);
            x23 = sin(q23);
            y23 = cos(q23);
            
            A1 = l2 * x11;
            A2 = - l2 * x21 * (1 + y23);
            A3 = l2 * x21 * x23;
            A4 = l2 * x11 * y12;
            A5 = l2 * y11;
            A6 = - l2 * y21 * (1 + y23);
            A7 = l2 * y21 * x23;
            A8 = l2 * y11 * y12 + l1;
            
            C1 = (A1^4*x12^4 + 4*A1^4*x12^3*x23 + 6*A1^4*x12^2*x23^2 - 2*A1^4*x12^2 + 4*A1^4*x12*x23^3 - 4*A1^4*x12*x23 + A1^4*x23^4 - 2*A1^4*x23^2 + A1^4 + 2*A1^2*A2^2*x12^2 + 4*A1^2*A2^2*x12*x23 + 2*A1^2*A2^2*x23^2 - 2*A1^2*A2^2 - 4*A1^2*A2*A4*x12^2 - 8*A1^2*A2*A4*x12*x23 - 4*A1^2*A2*A4*x23^2 + 4*A1^2*A2*A4 + 2*A1^2*A4^2*x12^2 + 4*A1^2*A4^2*x12*x23 + 2*A1^2*A4^2*x23^2 - 2*A1^2*A4^2 + 2*A1^2*A5^2*x12^4 + 8*A1^2*A5^2*x12^3*x23 + 12*A1^2*A5^2*x12^2*x23^2 - 4*A1^2*A5^2*x12^2 + 8*A1^2*A5^2*x12*x23^3 - 8*A1^2*A5^2*x12*x23 + 2*A1^2*A5^2*x23^4 - 4*A1^2*A5^2*x23^2 + 2*A1^2*A5^2 - 2*A1^2*A6^2*x12^2 - 4*A1^2*A6^2*x12*x23 - 2*A1^2*A6^2*x23^2 + 2*A1^2*A6^2 + 4*A1^2*A6*A8*x12^2 + 8*A1^2*A6*A8*x12*x23 + 4*A1^2*A6*A8*x23^2 - 4*A1^2*A6*A8 - 2*A1^2*A8^2*x12^2 - 4*A1^2*A8^2*x12*x23 - 2*A1^2*A8^2*x23^2 + 2*A1^2*A8^2 + 2*A1^2*l1^2*x12^2 + 4*A1^2*l1^2*x12*x23 + 2*A1^2*l1^2*x23^2 - 2*A1^2*l1^2 + 8*A1*A2*A5*A6*x12^2 + 16*A1*A2*A5*A6*x12*x23 + 8*A1*A2*A5*A6*x23^2 - 8*A1*A2*A5*A6 - 8*A1*A2*A5*A8*x12^2 - 16*A1*A2*A5*A8*x12*x23 - 8*A1*A2*A5*A8*x23^2 + 8*A1*A2*A5*A8 - 8*A1*A4*A5*A6*x12^2 - 16*A1*A4*A5*A6*x12*x23 - 8*A1*A4*A5*A6*x23^2 + 8*A1*A4*A5*A6 + 8*A1*A4*A5*A8*x12^2 + 16*A1*A4*A5*A8*x12*x23 + 8*A1*A4*A5*A8*x23^2 - 8*A1*A4*A5*A8 + A2^4 - 4*A2^3*A4 + 6*A2^2*A4^2 - 2*A2^2*A5^2*x12^2 - 4*A2^2*A5^2*x12*x23 - 2*A2^2*A5^2*x23^2 + 2*A2^2*A5^2 + 2*A2^2*A6^2 - 4*A2^2*A6*A8 + 2*A2^2*A8^2 - 2*A2^2*l1^2 - 4*A2*A4^3 + 4*A2*A4*A5^2*x12^2 + 8*A2*A4*A5^2*x12*x23 + 4*A2*A4*A5^2*x23^2 - 4*A2*A4*A5^2 - 4*A2*A4*A6^2 + 8*A2*A4*A6*A8 - 4*A2*A4*A8^2 + 4*A2*A4*l1^2 + A4^4 - 2*A4^2*A5^2*x12^2 - 4*A4^2*A5^2*x12*x23 - 2*A4^2*A5^2*x23^2 + 2*A4^2*A5^2 + 2*A4^2*A6^2 - 4*A4^2*A6*A8 + 2*A4^2*A8^2 - 2*A4^2*l1^2 + A5^4*x12^4 + 4*A5^4*x12^3*x23 + 6*A5^4*x12^2*x23^2 - 2*A5^4*x12^2 + 4*A5^4*x12*x23^3 - 4*A5^4*x12*x23 + A5^4*x23^4 - 2*A5^4*x23^2 + A5^4 + 2*A5^2*A6^2*x12^2 + 4*A5^2*A6^2*x12*x23 + 2*A5^2*A6^2*x23^2 - 2*A5^2*A6^2 - 4*A5^2*A6*A8*x12^2 - 8*A5^2*A6*A8*x12*x23 - 4*A5^2*A6*A8*x23^2 + 4*A5^2*A6*A8 + 2*A5^2*A8^2*x12^2 + 4*A5^2*A8^2*x12*x23 + 2*A5^2*A8^2*x23^2 - 2*A5^2*A8^2 + 2*A5^2*l1^2*x12^2 + 4*A5^2*l1^2*x12*x23 + 2*A5^2*l1^2*x23^2 - 2*A5^2*l1^2 + A6^4 - 4*A6^3*A8 + 6*A6^2*A8^2 - 2*A6^2*l1^2 - 4*A6*A8^3 + 4*A6*A8*l1^2 + A8^4 - 2*A8^2*l1^2 + l1^4);
            C2 = (8*A3*A4^3 - 8*A2^3*A3 - 8*A6^3*A7 + 8*A7*A8^3 + 8*A1^4*x12 + 8*A5^4*x12 + 8*A1^4*x23 + 8*A5^4*x23 - 8*A1^4*x12^3 - 8*A5^4*x12^3 - 8*A1^4*x23^3 - 8*A5^4*x23^3 - 8*A1^2*A2^2*x12 - 8*A1^2*A4^2*x12 + 16*A1^2*A5^2*x12 + 8*A1^2*A6^2*x12 + 8*A2^2*A5^2*x12 + 8*A1^2*A8^2*x12 + 8*A4^2*A5^2*x12 - 8*A5^2*A6^2*x12 - 8*A5^2*A8^2*x12 - 8*A1^2*A2^2*x23 - 8*A1^2*A4^2*x23 + 16*A1^2*A5^2*x23 + 8*A1^2*A6^2*x23 + 8*A2^2*A5^2*x23 + 8*A1^2*A8^2*x23 + 8*A4^2*A5^2*x23 - 8*A5^2*A6^2*x23 - 8*A5^2*A8^2*x23 - 8*A1^2*l1^2*x12 - 8*A5^2*l1^2*x12 - 8*A1^2*l1^2*x23 - 8*A5^2*l1^2*x23 - 24*A1^4*x12*x23^2 - 24*A1^4*x12^2*x23 - 24*A5^4*x12*x23^2 - 24*A5^4*x12^2*x23 - 8*A1^4*x12^3*y23 - 8*A5^4*x12^3*y23 - 8*A1^4*x23^3*y23 - 8*A5^4*x23^3*y23 - 16*A1^2*A5^2*x12^3 - 16*A1^2*A5^2*x23^3 + 8*A1^2*A2*A3 - 8*A1^2*A3*A4 - 24*A2*A3*A4^2 + 24*A2^2*A3*A4 - 8*A2*A3*A5^2 - 8*A2*A3*A6^2 + 8*A3*A4*A5^2 - 8*A2*A3*A8^2 + 8*A3*A4*A6^2 - 8*A1^2*A6*A7 + 8*A3*A4*A8^2 - 8*A2^2*A6*A7 + 8*A1^2*A7*A8 + 8*A2^2*A7*A8 - 8*A4^2*A6*A7 + 8*A5^2*A6*A7 + 8*A4^2*A7*A8 - 8*A5^2*A7*A8 - 24*A6*A7*A8^2 + 24*A6^2*A7*A8 + 8*A2*A3*l1^2 - 8*A3*A4*l1^2 + 8*A6*A7*l1^2 - 8*A7*A8*l1^2 + 8*A1^4*x12*y23 + 8*A5^4*x12*y23 + 8*A1^4*x23*y23 + 8*A5^4*x23*y23 - 8*A1^2*A2*A3*x12^2 + 8*A1^2*A3*A4*x12^2 + 8*A2*A3*A5^2*x12^2 - 8*A3*A4*A5^2*x12^2 + 8*A1^2*A6*A7*x12^2 - 8*A1^2*A7*A8*x12^2 - 8*A1^2*A2*A3*x23^2 - 8*A5^2*A6*A7*x12^2 + 8*A1^2*A3*A4*x23^2 + 8*A5^2*A7*A8*x12^2 + 8*A2*A3*A5^2*x23^2 - 8*A3*A4*A5^2*x23^2 + 8*A1^2*A6*A7*x23^2 - 8*A1^2*A7*A8*x23^2 - 8*A5^2*A6*A7*x23^2 + 8*A5^2*A7*A8*x23^2 + 16*A1*A2*A5*A7 + 16*A1*A3*A5*A6 - 16*A1*A3*A5*A8 - 16*A1*A4*A5*A7 + 16*A2*A3*A6*A8 + 16*A2*A4*A6*A7 - 16*A2*A4*A7*A8 - 16*A3*A4*A6*A8 - 8*A1^2*A2^2*x12*y23 - 8*A1^2*A4^2*x12*y23 + 16*A1^2*A5^2*x12*y23 + 8*A1^2*A6^2*x12*y23 + 8*A2^2*A5^2*x12*y23 + 8*A1^2*A8^2*x12*y23 + 8*A4^2*A5^2*x12*y23 - 8*A5^2*A6^2*x12*y23 - 8*A5^2*A8^2*x12*y23 - 8*A1^2*A2^2*x23*y23 - 8*A1^2*A4^2*x23*y23 + 16*A1^2*A5^2*x23*y23 + 8*A1^2*A6^2*x23*y23 + 8*A2^2*A5^2*x23*y23 + 8*A1^2*A8^2*x23*y23 + 8*A4^2*A5^2*x23*y23 - 8*A5^2*A6^2*x23*y23 - 8*A5^2*A8^2*x23*y23 - 8*A1^2*l1^2*x12*y23 - 8*A5^2*l1^2*x12*y23 - 8*A1^2*l1^2*x23*y23 - 8*A5^2*l1^2*x23*y23 - 24*A1^4*x12*x23^2*y23 - 24*A1^4*x12^2*x23*y23 - 24*A5^4*x12*x23^2*y23 - 24*A5^4*x12^2*x23*y23 - 48*A1^2*A5^2*x12*x23^2 - 48*A1^2*A5^2*x12^2*x23 - 16*A1^2*A5^2*x12^3*y23 - 16*A1^2*A5^2*x23^3*y23 + 16*A1^2*A2*A4*x12 - 16*A2*A4*A5^2*x12 - 16*A1^2*A6*A8*x12 + 16*A1^2*A2*A4*x23 + 16*A5^2*A6*A8*x12 - 16*A2*A4*A5^2*x23 - 16*A1^2*A6*A8*x23 + 16*A5^2*A6*A8*x23 - 16*A1*A2*A5*A7*x12^2 - 16*A1*A3*A5*A6*x12^2 + 16*A1*A3*A5*A8*x12^2 + 16*A1*A4*A5*A7*x12^2 - 16*A1*A2*A5*A7*x23^2 - 16*A1*A3*A5*A6*x23^2 + 16*A1*A3*A5*A8*x23^2 + 16*A1*A4*A5*A7*x23^2 - 48*A1^2*A5^2*x12*x23^2*y23 - 48*A1^2*A5^2*x12^2*x23*y23 - 16*A1^2*A2*A3*x12*x23 + 16*A1^2*A3*A4*x12*x23 + 16*A2*A3*A5^2*x12*x23 - 16*A3*A4*A5^2*x12*x23 + 16*A1^2*A6*A7*x12*x23 - 16*A1^2*A7*A8*x12*x23 - 16*A5^2*A6*A7*x12*x23 + 16*A5^2*A7*A8*x12*x23 + 16*A1^2*A2*A4*x12*y23 - 16*A2*A4*A5^2*x12*y23 - 16*A1^2*A6*A8*x12*y23 + 16*A1^2*A2*A4*x23*y23 + 16*A5^2*A6*A8*x12*y23 - 16*A2*A4*A5^2*x23*y23 - 16*A1^2*A6*A8*x23*y23 + 16*A5^2*A6*A8*x23*y23 - 32*A1*A2*A5*A6*x12 + 32*A1*A2*A5*A8*x12 + 32*A1*A4*A5*A6*x12 - 32*A1*A4*A5*A8*x12 - 32*A1*A2*A5*A6*x23 + 32*A1*A2*A5*A8*x23 + 32*A1*A4*A5*A6*x23 - 32*A1*A4*A5*A8*x23 - 32*A1*A2*A5*A7*x12*x23 - 32*A1*A3*A5*A6*x12*x23 + 32*A1*A3*A5*A8*x12*x23 + 32*A1*A4*A5*A7*x12*x23 - 32*A1*A2*A5*A6*x12*y23 + 32*A1*A2*A5*A8*x12*y23 + 32*A1*A4*A5*A6*x12*y23 - 32*A1*A4*A5*A8*x12*y23 - 32*A1*A2*A5*A6*x23*y23 + 32*A1*A2*A5*A8*x23*y23 + 32*A1*A4*A5*A6*x23*y23 - 32*A1*A4*A5*A8*x23*y23);
            C3 = (4*A1^4*x12^4 + 8*A1^4*x12^3*x23 + 24*A1^4*x12^2*y23^2 + 48*A1^4*x12^2*y23 + 16*A1^4*x12^2 - 8*A1^4*x12*x23^3 + 48*A1^4*x12*x23*y23^2 + 96*A1^4*x12*x23*y23 + 40*A1^4*x12*x23 - 4*A1^4*x23^4 + 24*A1^4*x23^2*y23^2 + 48*A1^4*x23^2*y23 + 24*A1^4*x23^2 - 8*A1^4*y23^2 - 16*A1^4*y23 - 4*A1^4 - 8*A1^2*A2^2*x12*x23 - 8*A1^2*A2^2*x23^2 + 8*A1^2*A2^2*y23^2 + 16*A1^2*A2^2*y23 + 8*A1^2*A2^2 + 32*A1^2*A2*A3*x12*y23 + 32*A1^2*A2*A3*x12 + 32*A1^2*A2*A3*x23*y23 + 32*A1^2*A2*A3*x23 - 8*A1^2*A2*A4*x12^2 + 8*A1^2*A2*A4*x23^2 - 16*A1^2*A2*A4*y23^2 - 32*A1^2*A2*A4*y23 - 8*A1^2*A2*A4 + 8*A1^2*A3^2*x12^2 + 16*A1^2*A3^2*x12*x23 + 8*A1^2*A3^2*x23^2 - 8*A1^2*A3^2 - 32*A1^2*A3*A4*x12*y23 - 32*A1^2*A3*A4*x12 - 32*A1^2*A3*A4*x23*y23 - 32*A1^2*A3*A4*x23 + 8*A1^2*A4^2*x12^2 + 8*A1^2*A4^2*x12*x23 + 8*A1^2*A4^2*y23^2 + 16*A1^2*A4^2*y23 + 8*A1^2*A5^2*x12^4 + 16*A1^2*A5^2*x12^3*x23 + 48*A1^2*A5^2*x12^2*y23^2 + 96*A1^2*A5^2*x12^2*y23 + 32*A1^2*A5^2*x12^2 - 16*A1^2*A5^2*x12*x23^3 + 96*A1^2*A5^2*x12*x23*y23^2 + 192*A1^2*A5^2*x12*x23*y23 + 80*A1^2*A5^2*x12*x23 - 8*A1^2*A5^2*x23^4 + 48*A1^2*A5^2*x23^2*y23^2 + 96*A1^2*A5^2*x23^2*y23 + 48*A1^2*A5^2*x23^2 - 16*A1^2*A5^2*y23^2 - 32*A1^2*A5^2*y23 - 8*A1^2*A5^2 + 8*A1^2*A6^2*x12*x23 + 8*A1^2*A6^2*x23^2 - 8*A1^2*A6^2*y23^2 - 16*A1^2*A6^2*y23 - 8*A1^2*A6^2 - 32*A1^2*A6*A7*x12*y23 - 32*A1^2*A6*A7*x12 - 32*A1^2*A6*A7*x23*y23 - 32*A1^2*A6*A7*x23 + 8*A1^2*A6*A8*x12^2 - 8*A1^2*A6*A8*x23^2 + 16*A1^2*A6*A8*y23^2 + 32*A1^2*A6*A8*y23 + 8*A1^2*A6*A8 - 8*A1^2*A7^2*x12^2 - 16*A1^2*A7^2*x12*x23 - 8*A1^2*A7^2*x23^2 + 8*A1^2*A7^2 + 32*A1^2*A7*A8*x12*y23 + 32*A1^2*A7*A8*x12 + 32*A1^2*A7*A8*x23*y23 + 32*A1^2*A7*A8*x23 - 8*A1^2*A8^2*x12^2 - 8*A1^2*A8^2*x12*x23 - 8*A1^2*A8^2*y23^2 - 16*A1^2*A8^2*y23 + 8*A1^2*l1^2*x12^2 + 8*A1^2*l1^2*x12*x23 + 8*A1^2*l1^2*y23^2 + 16*A1^2*l1^2*y23 - 32*A1*A2*A5*A6*x12*x23 - 32*A1*A2*A5*A6*x23^2 + 32*A1*A2*A5*A6*y23^2 + 64*A1*A2*A5*A6*y23 + 32*A1*A2*A5*A6 + 64*A1*A2*A5*A7*x12*y23 + 64*A1*A2*A5*A7*x12 + 64*A1*A2*A5*A7*x23*y23 + 64*A1*A2*A5*A7*x23 - 16*A1*A2*A5*A8*x12^2 + 16*A1*A2*A5*A8*x23^2 - 32*A1*A2*A5*A8*y23^2 - 64*A1*A2*A5*A8*y23 - 16*A1*A2*A5*A8 + 64*A1*A3*A5*A6*x12*y23 + 64*A1*A3*A5*A6*x12 + 64*A1*A3*A5*A6*x23*y23 + 64*A1*A3*A5*A6*x23 + 32*A1*A3*A5*A7*x12^2 + 64*A1*A3*A5*A7*x12*x23 + 32*A1*A3*A5*A7*x23^2 - 32*A1*A3*A5*A7 - 64*A1*A3*A5*A8*x12*y23 - 64*A1*A3*A5*A8*x12 - 64*A1*A3*A5*A8*x23*y23 - 64*A1*A3*A5*A8*x23 - 16*A1*A4*A5*A6*x12^2 + 16*A1*A4*A5*A6*x23^2 - 32*A1*A4*A5*A6*y23^2 - 64*A1*A4*A5*A6*y23 - 16*A1*A4*A5*A6 - 64*A1*A4*A5*A7*x12*y23 - 64*A1*A4*A5*A7*x12 - 64*A1*A4*A5*A7*x23*y23 - 64*A1*A4*A5*A7*x23 + 32*A1*A4*A5*A8*x12^2 + 32*A1*A4*A5*A8*x12*x23 + 32*A1*A4*A5*A8*y23^2 + 64*A1*A4*A5*A8*y23 - 4*A2^4 + 8*A2^3*A4 + 24*A2^2*A3^2 + 8*A2^2*A5^2*x12*x23 + 8*A2^2*A5^2*x23^2 - 8*A2^2*A5^2*y23^2 - 16*A2^2*A5^2*y23 - 8*A2^2*A5^2 - 8*A2^2*A6^2 + 8*A2^2*A6*A8 + 8*A2^2*A7^2 - 48*A2*A3^2*A4 - 32*A2*A3*A5^2*x12*y23 - 32*A2*A3*A5^2*x12 - 32*A2*A3*A5^2*x23*y23 - 32*A2*A3*A5^2*x23 + 32*A2*A3*A6*A7 - 32*A2*A3*A7*A8 - 8*A2*A4^3 + 8*A2*A4*A5^2*x12^2 - 8*A2*A4*A5^2*x23^2 + 16*A2*A4*A5^2*y23^2 + 32*A2*A4*A5^2*y23 + 8*A2*A4*A5^2 + 8*A2*A4*A6^2 - 16*A2*A4*A7^2 - 8*A2*A4*A8^2 + 8*A2*A4*l1^2 + 24*A3^2*A4^2 - 8*A3^2*A5^2*x12^2 - 16*A3^2*A5^2*x12*x23 - 8*A3^2*A5^2*x23^2 + 8*A3^2*A5^2 + 8*A3^2*A6^2 - 16*A3^2*A6*A8 + 8*A3^2*A8^2 - 8*A3^2*l1^2 + 32*A3*A4*A5^2*x12*y23 + 32*A3*A4*A5^2*x12 + 32*A3*A4*A5^2*x23*y23 + 32*A3*A4*A5^2*x23 - 32*A3*A4*A6*A7 + 32*A3*A4*A7*A8 + 4*A4^4 - 8*A4^2*A5^2*x12^2 - 8*A4^2*A5^2*x12*x23 - 8*A4^2*A5^2*y23^2 - 16*A4^2*A5^2*y23 - 8*A4^2*A6*A8 + 8*A4^2*A7^2 + 8*A4^2*A8^2 - 8*A4^2*l1^2 + 4*A5^4*x12^4 + 8*A5^4*x12^3*x23 + 24*A5^4*x12^2*y23^2 + 48*A5^4*x12^2*y23 + 16*A5^4*x12^2 - 8*A5^4*x12*x23^3 + 48*A5^4*x12*x23*y23^2 + 96*A5^4*x12*x23*y23 + 40*A5^4*x12*x23 - 4*A5^4*x23^4 + 24*A5^4*x23^2*y23^2 + 48*A5^4*x23^2*y23 + 24*A5^4*x23^2 - 8*A5^4*y23^2 - 16*A5^4*y23 - 4*A5^4 - 8*A5^2*A6^2*x12*x23 - 8*A5^2*A6^2*x23^2 + 8*A5^2*A6^2*y23^2 + 16*A5^2*A6^2*y23 + 8*A5^2*A6^2 + 32*A5^2*A6*A7*x12*y23 + 32*A5^2*A6*A7*x12 + 32*A5^2*A6*A7*x23*y23 + 32*A5^2*A6*A7*x23 - 8*A5^2*A6*A8*x12^2 + 8*A5^2*A6*A8*x23^2 - 16*A5^2*A6*A8*y23^2 - 32*A5^2*A6*A8*y23 - 8*A5^2*A6*A8 + 8*A5^2*A7^2*x12^2 + 16*A5^2*A7^2*x12*x23 + 8*A5^2*A7^2*x23^2 - 8*A5^2*A7^2 - 32*A5^2*A7*A8*x12*y23 - 32*A5^2*A7*A8*x12 - 32*A5^2*A7*A8*x23*y23 - 32*A5^2*A7*A8*x23 + 8*A5^2*A8^2*x12^2 + 8*A5^2*A8^2*x12*x23 + 8*A5^2*A8^2*y23^2 + 16*A5^2*A8^2*y23 + 8*A5^2*l1^2*x12^2 + 8*A5^2*l1^2*x12*x23 + 8*A5^2*l1^2*y23^2 + 16*A5^2*l1^2*y23 - 4*A6^4 + 8*A6^3*A8 + 24*A6^2*A7^2 - 48*A6*A7^2*A8 - 8*A6*A8^3 + 8*A6*A8*l1^2 + 24*A7^2*A8^2 - 8*A7^2*l1^2 + 4*A8^4 - 8*A8^2*l1^2 + 4*l1^4);
            C4 = (24*A1^4*x12*x23^2*y23 - 24*A1^4*x12^3 - 24*A1^4*x12^2*x23*y23 - 24*A1^4*x12^2*x23 - 24*A1^4*x12^3*y23 + 24*A1^4*x12*x23^2 - 32*A1^4*x12*y23^3 - 96*A1^4*x12*y23^2 - 72*A1^4*x12*y23 - 8*A1^4*x12 + 24*A1^4*x23^3*y23 + 24*A1^4*x23^3 - 32*A1^4*x23*y23^3 - 96*A1^4*x23*y23^2 - 88*A1^4*x23*y23 - 24*A1^4*x23 + 8*A1^2*A2^2*x12*y23 + 8*A1^2*A2^2*x12 + 24*A1^2*A2^2*x23*y23 + 24*A1^2*A2^2*x23 - 8*A1^2*A2*A3*x12^2 + 16*A1^2*A2*A3*x12*x23 + 24*A1^2*A2*A3*x23^2 - 32*A1^2*A2*A3*y23^2 - 64*A1^2*A2*A3*y23 - 24*A1^2*A2*A3 + 16*A1^2*A2*A4*x12*y23 + 16*A1^2*A2*A4*x12 - 16*A1^2*A2*A4*x23*y23 - 16*A1^2*A2*A4*x23 - 32*A1^2*A3^2*x12*y23 - 32*A1^2*A3^2*x12 - 32*A1^2*A3^2*x23*y23 - 32*A1^2*A3^2*x23 + 24*A1^2*A3*A4*x12^2 + 16*A1^2*A3*A4*x12*x23 - 8*A1^2*A3*A4*x23^2 + 32*A1^2*A3*A4*y23^2 + 64*A1^2*A3*A4*y23 + 8*A1^2*A3*A4 - 24*A1^2*A4^2*x12*y23 - 24*A1^2*A4^2*x12 - 8*A1^2*A4^2*x23*y23 - 8*A1^2*A4^2*x23 - 48*A1^2*A5^2*x12^3*y23 - 48*A1^2*A5^2*x12^3 - 48*A1^2*A5^2*x12^2*x23*y23 - 48*A1^2*A5^2*x12^2*x23 + 48*A1^2*A5^2*x12*x23^2*y23 + 48*A1^2*A5^2*x12*x23^2 - 64*A1^2*A5^2*x12*y23^3 - 192*A1^2*A5^2*x12*y23^2 - 144*A1^2*A5^2*x12*y23 - 16*A1^2*A5^2*x12 + 48*A1^2*A5^2*x23^3*y23 + 48*A1^2*A5^2*x23^3 - 64*A1^2*A5^2*x23*y23^3 - 192*A1^2*A5^2*x23*y23^2 - 176*A1^2*A5^2*x23*y23 - 48*A1^2*A5^2*x23 - 8*A1^2*A6^2*x12*y23 - 8*A1^2*A6^2*x12 - 24*A1^2*A6^2*x23*y23 - 24*A1^2*A6^2*x23 + 8*A1^2*A6*A7*x12^2 - 16*A1^2*A6*A7*x12*x23 - 24*A1^2*A6*A7*x23^2 + 32*A1^2*A6*A7*y23^2 + 64*A1^2*A6*A7*y23 + 24*A1^2*A6*A7 - 16*A1^2*A6*A8*x12*y23 - 16*A1^2*A6*A8*x12 + 16*A1^2*A6*A8*x23*y23 + 16*A1^2*A6*A8*x23 + 32*A1^2*A7^2*x12*y23 + 32*A1^2*A7^2*x12 + 32*A1^2*A7^2*x23*y23 + 32*A1^2*A7^2*x23 - 24*A1^2*A7*A8*x12^2 - 16*A1^2*A7*A8*x12*x23 + 8*A1^2*A7*A8*x23^2 - 32*A1^2*A7*A8*y23^2 - 64*A1^2*A7*A8*y23 - 8*A1^2*A7*A8 + 24*A1^2*A8^2*x12*y23 + 24*A1^2*A8^2*x12 + 8*A1^2*A8^2*x23*y23 + 8*A1^2*A8^2*x23 - 24*A1^2*l1^2*x12*y23 - 24*A1^2*l1^2*x12 - 8*A1^2*l1^2*x23*y23 - 8*A1^2*l1^2*x23 + 32*A1*A2*A5*A6*x12*y23 + 32*A1*A2*A5*A6*x12 + 96*A1*A2*A5*A6*x23*y23 + 96*A1*A2*A5*A6*x23 - 16*A1*A2*A5*A7*x12^2 + 32*A1*A2*A5*A7*x12*x23 + 48*A1*A2*A5*A7*x23^2 - 64*A1*A2*A5*A7*y23^2 - 128*A1*A2*A5*A7*y23 - 48*A1*A2*A5*A7 + 32*A1*A2*A5*A8*x12*y23 + 32*A1*A2*A5*A8*x12 - 32*A1*A2*A5*A8*x23*y23 - 32*A1*A2*A5*A8*x23 - 16*A1*A3*A5*A6*x12^2 + 32*A1*A3*A5*A6*x12*x23 + 48*A1*A3*A5*A6*x23^2 - 64*A1*A3*A5*A6*y23^2 - 128*A1*A3*A5*A6*y23 - 48*A1*A3*A5*A6 - 128*A1*A3*A5*A7*x12*y23 - 128*A1*A3*A5*A7*x12 - 128*A1*A3*A5*A7*x23*y23 - 128*A1*A3*A5*A7*x23 + 48*A1*A3*A5*A8*x12^2 + 32*A1*A3*A5*A8*x12*x23 - 16*A1*A3*A5*A8*x23^2 + 64*A1*A3*A5*A8*y23^2 + 128*A1*A3*A5*A8*y23 + 16*A1*A3*A5*A8 + 32*A1*A4*A5*A6*x12*y23 + 32*A1*A4*A5*A6*x12 - 32*A1*A4*A5*A6*x23*y23 - 32*A1*A4*A5*A6*x23 + 48*A1*A4*A5*A7*x12^2 + 32*A1*A4*A5*A7*x12*x23 - 16*A1*A4*A5*A7*x23^2 + 64*A1*A4*A5*A7*y23^2 + 128*A1*A4*A5*A7*y23 + 16*A1*A4*A5*A7 - 96*A1*A4*A5*A8*x12*y23 - 96*A1*A4*A5*A8*x12 - 32*A1*A4*A5*A8*x23*y23 - 32*A1*A4*A5*A8*x23 + 24*A2^3*A3 - 24*A2^2*A3*A4 - 8*A2^2*A5^2*x12*y23 - 8*A2^2*A5^2*x12 - 24*A2^2*A5^2*x23*y23 - 24*A2^2*A5^2*x23 + 24*A2^2*A6*A7 - 8*A2^2*A7*A8 - 32*A2*A3^3 - 24*A2*A3*A4^2 + 8*A2*A3*A5^2*x12^2 - 16*A2*A3*A5^2*x12*x23 - 24*A2*A3*A5^2*x23^2 + 32*A2*A3*A5^2*y23^2 + 64*A2*A3*A5^2*y23 + 24*A2*A3*A5^2 + 24*A2*A3*A6^2 - 16*A2*A3*A6*A8 - 32*A2*A3*A7^2 - 8*A2*A3*A8^2 + 8*A2*A3*l1^2 - 16*A2*A4*A5^2*x12*y23 - 16*A2*A4*A5^2*x12 + 16*A2*A4*A5^2*x23*y23 + 16*A2*A4*A5^2*x23 - 16*A2*A4*A6*A7 - 16*A2*A4*A7*A8 + 32*A3^3*A4 + 32*A3^2*A5^2*x12*y23 + 32*A3^2*A5^2*x12 + 32*A3^2*A5^2*x23*y23 + 32*A3^2*A5^2*x23 - 32*A3^2*A6*A7 + 32*A3^2*A7*A8 + 24*A3*A4^3 - 24*A3*A4*A5^2*x12^2 - 16*A3*A4*A5^2*x12*x23 + 8*A3*A4*A5^2*x23^2 - 32*A3*A4*A5^2*y23^2 - 64*A3*A4*A5^2*y23 - 8*A3*A4*A5^2 - 8*A3*A4*A6^2 - 16*A3*A4*A6*A8 + 32*A3*A4*A7^2 + 24*A3*A4*A8^2 - 24*A3*A4*l1^2 + 24*A4^2*A5^2*x12*y23 + 24*A4^2*A5^2*x12 + 8*A4^2*A5^2*x23*y23 + 8*A4^2*A5^2*x23 - 8*A4^2*A6*A7 + 24*A4^2*A7*A8 - 24*A5^4*x12^3*y23 - 24*A5^4*x12^3 - 24*A5^4*x12^2*x23*y23 - 24*A5^4*x12^2*x23 + 24*A5^4*x12*x23^2*y23 + 24*A5^4*x12*x23^2 - 32*A5^4*x12*y23^3 - 96*A5^4*x12*y23^2 - 72*A5^4*x12*y23 - 8*A5^4*x12 + 24*A5^4*x23^3*y23 + 24*A5^4*x23^3 - 32*A5^4*x23*y23^3 - 96*A5^4*x23*y23^2 - 88*A5^4*x23*y23 - 24*A5^4*x23 + 8*A5^2*A6^2*x12*y23 + 8*A5^2*A6^2*x12 + 24*A5^2*A6^2*x23*y23 + 24*A5^2*A6^2*x23 - 8*A5^2*A6*A7*x12^2 + 16*A5^2*A6*A7*x12*x23 + 24*A5^2*A6*A7*x23^2 - 32*A5^2*A6*A7*y23^2 - 64*A5^2*A6*A7*y23 - 24*A5^2*A6*A7 + 16*A5^2*A6*A8*x12*y23 + 16*A5^2*A6*A8*x12 - 16*A5^2*A6*A8*x23*y23 - 16*A5^2*A6*A8*x23 - 32*A5^2*A7^2*x12*y23 - 32*A5^2*A7^2*x12 - 32*A5^2*A7^2*x23*y23 - 32*A5^2*A7^2*x23 + 24*A5^2*A7*A8*x12^2 + 16*A5^2*A7*A8*x12*x23 - 8*A5^2*A7*A8*x23^2 + 32*A5^2*A7*A8*y23^2 + 64*A5^2*A7*A8*y23 + 8*A5^2*A7*A8 - 24*A5^2*A8^2*x12*y23 - 24*A5^2*A8^2*x12 - 8*A5^2*A8^2*x23*y23 - 8*A5^2*A8^2*x23 - 24*A5^2*l1^2*x12*y23 - 24*A5^2*l1^2*x12 - 8*A5^2*l1^2*x23*y23 - 8*A5^2*l1^2*x23 + 24*A6^3*A7 - 24*A6^2*A7*A8 - 32*A6*A7^3 - 24*A6*A7*A8^2 + 8*A6*A7*l1^2 + 32*A7^3*A8 + 24*A7*A8^3 - 24*A7*A8*l1^2);
            C5 = (6*A1^4*x12^4 - 12*A1^4*x12^2*x23^2 + 48*A1^4*x12^2*y23^2 + 96*A1^4*x12^2*y23 + 36*A1^4*x12^2 + 6*A1^4*x23^4 - 48*A1^4*x23^2*y23^2 - 96*A1^4*x23^2*y23 - 44*A1^4*x23^2 + 16*A1^4*y23^4 + 64*A1^4*y23^3 + 80*A1^4*y23^2 + 32*A1^4*y23 + 6*A1^4 - 4*A1^2*A2^2*x12^2 + 12*A1^2*A2^2*x23^2 - 16*A1^2*A2^2*y23^2 - 32*A1^2*A2^2*y23 - 12*A1^2*A2^2 - 64*A1^2*A2*A3*x23*y23 - 64*A1^2*A2*A3*x23 + 16*A1^2*A2*A4*x12*x23 + 16*A1^2*A3^2*x12^2 - 16*A1^2*A3^2*x23^2 + 32*A1^2*A3^2*y23^2 + 64*A1^2*A3^2*y23 + 16*A1^2*A3^2 - 64*A1^2*A3*A4*x12*y23 - 64*A1^2*A3*A4*x12 + 12*A1^2*A4^2*x12^2 - 4*A1^2*A4^2*x23^2 + 16*A1^2*A4^2*y23^2 + 32*A1^2*A4^2*y23 + 4*A1^2*A4^2 + 12*A1^2*A5^2*x12^4 - 24*A1^2*A5^2*x12^2*x23^2 + 96*A1^2*A5^2*x12^2*y23^2 + 192*A1^2*A5^2*x12^2*y23 + 72*A1^2*A5^2*x12^2 + 12*A1^2*A5^2*x23^4 - 96*A1^2*A5^2*x23^2*y23^2 - 192*A1^2*A5^2*x23^2*y23 - 88*A1^2*A5^2*x23^2 + 32*A1^2*A5^2*y23^4 + 128*A1^2*A5^2*y23^3 + 160*A1^2*A5^2*y23^2 + 64*A1^2*A5^2*y23 + 12*A1^2*A5^2 + 4*A1^2*A6^2*x12^2 - 12*A1^2*A6^2*x23^2 + 16*A1^2*A6^2*y23^2 + 32*A1^2*A6^2*y23 + 12*A1^2*A6^2 + 64*A1^2*A6*A7*x23*y23 + 64*A1^2*A6*A7*x23 - 16*A1^2*A6*A8*x12*x23 - 16*A1^2*A7^2*x12^2 + 16*A1^2*A7^2*x23^2 - 32*A1^2*A7^2*y23^2 - 64*A1^2*A7^2*y23 - 16*A1^2*A7^2 + 64*A1^2*A7*A8*x12*y23 + 64*A1^2*A7*A8*x12 - 12*A1^2*A8^2*x12^2 + 4*A1^2*A8^2*x23^2 - 16*A1^2*A8^2*y23^2 - 32*A1^2*A8^2*y23 - 4*A1^2*A8^2 + 12*A1^2*l1^2*x12^2 - 4*A1^2*l1^2*x23^2 + 16*A1^2*l1^2*y23^2 + 32*A1^2*l1^2*y23 + 4*A1^2*l1^2 - 16*A1*A2*A5*A6*x12^2 + 48*A1*A2*A5*A6*x23^2 - 64*A1*A2*A5*A6*y23^2 - 128*A1*A2*A5*A6*y23 - 48*A1*A2*A5*A6 - 128*A1*A2*A5*A7*x23*y23 - 128*A1*A2*A5*A7*x23 + 32*A1*A2*A5*A8*x12*x23 - 128*A1*A3*A5*A6*x23*y23 - 128*A1*A3*A5*A6*x23 + 64*A1*A3*A5*A7*x12^2 - 64*A1*A3*A5*A7*x23^2 + 128*A1*A3*A5*A7*y23^2 + 256*A1*A3*A5*A7*y23 + 64*A1*A3*A5*A7 - 128*A1*A3*A5*A8*x12*y23 - 128*A1*A3*A5*A8*x12 + 32*A1*A4*A5*A6*x12*x23 - 128*A1*A4*A5*A7*x12*y23 - 128*A1*A4*A5*A7*x12 + 48*A1*A4*A5*A8*x12^2 - 16*A1*A4*A5*A8*x23^2 + 64*A1*A4*A5*A8*y23^2 + 128*A1*A4*A5*A8*y23 + 16*A1*A4*A5*A8 + 6*A2^4 - 48*A2^2*A3^2 - 12*A2^2*A4^2 + 4*A2^2*A5^2*x12^2 - 12*A2^2*A5^2*x23^2 + 16*A2^2*A5^2*y23^2 + 32*A2^2*A5^2*y23 + 12*A2^2*A5^2 + 12*A2^2*A6^2 - 16*A2^2*A7^2 - 4*A2^2*A8^2 + 4*A2^2*l1^2 + 64*A2*A3*A5^2*x23*y23 + 64*A2*A3*A5^2*x23 - 64*A2*A3*A6*A7 - 16*A2*A4*A5^2*x12*x23 - 16*A2*A4*A6*A8 + 16*A3^4 + 48*A3^2*A4^2 - 16*A3^2*A5^2*x12^2 + 16*A3^2*A5^2*x23^2 - 32*A3^2*A5^2*y23^2 - 64*A3^2*A5^2*y23 - 16*A3^2*A5^2 - 16*A3^2*A6^2 + 32*A3^2*A7^2 + 16*A3^2*A8^2 - 16*A3^2*l1^2 + 64*A3*A4*A5^2*x12*y23 + 64*A3*A4*A5^2*x12 + 64*A3*A4*A7*A8 + 6*A4^4 - 12*A4^2*A5^2*x12^2 + 4*A4^2*A5^2*x23^2 - 16*A4^2*A5^2*y23^2 - 32*A4^2*A5^2*y23 - 4*A4^2*A5^2 - 4*A4^2*A6^2 + 16*A4^2*A7^2 + 12*A4^2*A8^2 - 12*A4^2*l1^2 + 6*A5^4*x12^4 - 12*A5^4*x12^2*x23^2 + 48*A5^4*x12^2*y23^2 + 96*A5^4*x12^2*y23 + 36*A5^4*x12^2 + 6*A5^4*x23^4 - 48*A5^4*x23^2*y23^2 - 96*A5^4*x23^2*y23 - 44*A5^4*x23^2 + 16*A5^4*y23^4 + 64*A5^4*y23^3 + 80*A5^4*y23^2 + 32*A5^4*y23 + 6*A5^4 - 4*A5^2*A6^2*x12^2 + 12*A5^2*A6^2*x23^2 - 16*A5^2*A6^2*y23^2 - 32*A5^2*A6^2*y23 - 12*A5^2*A6^2 - 64*A5^2*A6*A7*x23*y23 - 64*A5^2*A6*A7*x23 + 16*A5^2*A6*A8*x12*x23 + 16*A5^2*A7^2*x12^2 - 16*A5^2*A7^2*x23^2 + 32*A5^2*A7^2*y23^2 + 64*A5^2*A7^2*y23 + 16*A5^2*A7^2 - 64*A5^2*A7*A8*x12*y23 - 64*A5^2*A7*A8*x12 + 12*A5^2*A8^2*x12^2 - 4*A5^2*A8^2*x23^2 + 16*A5^2*A8^2*y23^2 + 32*A5^2*A8^2*y23 + 4*A5^2*A8^2 + 12*A5^2*l1^2*x12^2 - 4*A5^2*l1^2*x23^2 + 16*A5^2*l1^2*y23^2 + 32*A5^2*l1^2*y23 + 4*A5^2*l1^2 + 6*A6^4 - 48*A6^2*A7^2 - 12*A6^2*A8^2 + 4*A6^2*l1^2 + 16*A7^4 + 48*A7^2*A8^2 - 16*A7^2*l1^2 + 6*A8^4 - 12*A8^2*l1^2 + 6*l1^4);
            C6 = (24*A1^4*x12^2*x23*y23 - 24*A1^4*x12^3 - 24*A1^4*x12^3*y23 + 24*A1^4*x12^2*x23 + 24*A1^4*x12*x23^2*y23 + 24*A1^4*x12*x23^2 - 32*A1^4*x12*y23^3 - 96*A1^4*x12*y23^2 - 72*A1^4*x12*y23 - 8*A1^4*x12 - 24*A1^4*x23^3*y23 - 24*A1^4*x23^3 + 32*A1^4*x23*y23^3 + 96*A1^4*x23*y23^2 + 88*A1^4*x23*y23 + 24*A1^4*x23 + 8*A1^2*A2^2*x12*y23 + 8*A1^2*A2^2*x12 - 24*A1^2*A2^2*x23*y23 - 24*A1^2*A2^2*x23 + 8*A1^2*A2*A3*x12^2 + 16*A1^2*A2*A3*x12*x23 - 24*A1^2*A2*A3*x23^2 + 32*A1^2*A2*A3*y23^2 + 64*A1^2*A2*A3*y23 + 24*A1^2*A2*A3 - 16*A1^2*A2*A4*x12*y23 - 16*A1^2*A2*A4*x12 - 16*A1^2*A2*A4*x23*y23 - 16*A1^2*A2*A4*x23 - 32*A1^2*A3^2*x12*y23 - 32*A1^2*A3^2*x12 + 32*A1^2*A3^2*x23*y23 + 32*A1^2*A3^2*x23 + 24*A1^2*A3*A4*x12^2 - 16*A1^2*A3*A4*x12*x23 - 8*A1^2*A3*A4*x23^2 + 32*A1^2*A3*A4*y23^2 + 64*A1^2*A3*A4*y23 + 8*A1^2*A3*A4 - 24*A1^2*A4^2*x12*y23 - 24*A1^2*A4^2*x12 + 8*A1^2*A4^2*x23*y23 + 8*A1^2*A4^2*x23 - 48*A1^2*A5^2*x12^3*y23 - 48*A1^2*A5^2*x12^3 + 48*A1^2*A5^2*x12^2*x23*y23 + 48*A1^2*A5^2*x12^2*x23 + 48*A1^2*A5^2*x12*x23^2*y23 + 48*A1^2*A5^2*x12*x23^2 - 64*A1^2*A5^2*x12*y23^3 - 192*A1^2*A5^2*x12*y23^2 - 144*A1^2*A5^2*x12*y23 - 16*A1^2*A5^2*x12 - 48*A1^2*A5^2*x23^3*y23 - 48*A1^2*A5^2*x23^3 + 64*A1^2*A5^2*x23*y23^3 + 192*A1^2*A5^2*x23*y23^2 + 176*A1^2*A5^2*x23*y23 + 48*A1^2*A5^2*x23 - 8*A1^2*A6^2*x12*y23 - 8*A1^2*A6^2*x12 + 24*A1^2*A6^2*x23*y23 + 24*A1^2*A6^2*x23 - 8*A1^2*A6*A7*x12^2 - 16*A1^2*A6*A7*x12*x23 + 24*A1^2*A6*A7*x23^2 - 32*A1^2*A6*A7*y23^2 - 64*A1^2*A6*A7*y23 - 24*A1^2*A6*A7 + 16*A1^2*A6*A8*x12*y23 + 16*A1^2*A6*A8*x12 + 16*A1^2*A6*A8*x23*y23 + 16*A1^2*A6*A8*x23 + 32*A1^2*A7^2*x12*y23 + 32*A1^2*A7^2*x12 - 32*A1^2*A7^2*x23*y23 - 32*A1^2*A7^2*x23 - 24*A1^2*A7*A8*x12^2 + 16*A1^2*A7*A8*x12*x23 + 8*A1^2*A7*A8*x23^2 - 32*A1^2*A7*A8*y23^2 - 64*A1^2*A7*A8*y23 - 8*A1^2*A7*A8 + 24*A1^2*A8^2*x12*y23 + 24*A1^2*A8^2*x12 - 8*A1^2*A8^2*x23*y23 - 8*A1^2*A8^2*x23 - 24*A1^2*l1^2*x12*y23 - 24*A1^2*l1^2*x12 + 8*A1^2*l1^2*x23*y23 + 8*A1^2*l1^2*x23 + 32*A1*A2*A5*A6*x12*y23 + 32*A1*A2*A5*A6*x12 - 96*A1*A2*A5*A6*x23*y23 - 96*A1*A2*A5*A6*x23 + 16*A1*A2*A5*A7*x12^2 + 32*A1*A2*A5*A7*x12*x23 - 48*A1*A2*A5*A7*x23^2 + 64*A1*A2*A5*A7*y23^2 + 128*A1*A2*A5*A7*y23 + 48*A1*A2*A5*A7 - 32*A1*A2*A5*A8*x12*y23 - 32*A1*A2*A5*A8*x12 - 32*A1*A2*A5*A8*x23*y23 - 32*A1*A2*A5*A8*x23 + 16*A1*A3*A5*A6*x12^2 + 32*A1*A3*A5*A6*x12*x23 - 48*A1*A3*A5*A6*x23^2 + 64*A1*A3*A5*A6*y23^2 + 128*A1*A3*A5*A6*y23 + 48*A1*A3*A5*A6 - 128*A1*A3*A5*A7*x12*y23 - 128*A1*A3*A5*A7*x12 + 128*A1*A3*A5*A7*x23*y23 + 128*A1*A3*A5*A7*x23 + 48*A1*A3*A5*A8*x12^2 - 32*A1*A3*A5*A8*x12*x23 - 16*A1*A3*A5*A8*x23^2 + 64*A1*A3*A5*A8*y23^2 + 128*A1*A3*A5*A8*y23 + 16*A1*A3*A5*A8 - 32*A1*A4*A5*A6*x12*y23 - 32*A1*A4*A5*A6*x12 - 32*A1*A4*A5*A6*x23*y23 - 32*A1*A4*A5*A6*x23 + 48*A1*A4*A5*A7*x12^2 - 32*A1*A4*A5*A7*x12*x23 - 16*A1*A4*A5*A7*x23^2 + 64*A1*A4*A5*A7*y23^2 + 128*A1*A4*A5*A7*y23 + 16*A1*A4*A5*A7 - 96*A1*A4*A5*A8*x12*y23 - 96*A1*A4*A5*A8*x12 + 32*A1*A4*A5*A8*x23*y23 + 32*A1*A4*A5*A8*x23 - 24*A2^3*A3 - 24*A2^2*A3*A4 - 8*A2^2*A5^2*x12*y23 - 8*A2^2*A5^2*x12 + 24*A2^2*A5^2*x23*y23 + 24*A2^2*A5^2*x23 - 24*A2^2*A6*A7 - 8*A2^2*A7*A8 + 32*A2*A3^3 + 24*A2*A3*A4^2 - 8*A2*A3*A5^2*x12^2 - 16*A2*A3*A5^2*x12*x23 + 24*A2*A3*A5^2*x23^2 - 32*A2*A3*A5^2*y23^2 - 64*A2*A3*A5^2*y23 - 24*A2*A3*A5^2 - 24*A2*A3*A6^2 - 16*A2*A3*A6*A8 + 32*A2*A3*A7^2 + 8*A2*A3*A8^2 - 8*A2*A3*l1^2 + 16*A2*A4*A5^2*x12*y23 + 16*A2*A4*A5^2*x12 + 16*A2*A4*A5^2*x23*y23 + 16*A2*A4*A5^2*x23 - 16*A2*A4*A6*A7 + 16*A2*A4*A7*A8 + 32*A3^3*A4 + 32*A3^2*A5^2*x12*y23 + 32*A3^2*A5^2*x12 - 32*A3^2*A5^2*x23*y23 - 32*A3^2*A5^2*x23 + 32*A3^2*A6*A7 + 32*A3^2*A7*A8 + 24*A3*A4^3 - 24*A3*A4*A5^2*x12^2 + 16*A3*A4*A5^2*x12*x23 + 8*A3*A4*A5^2*x23^2 - 32*A3*A4*A5^2*y23^2 - 64*A3*A4*A5^2*y23 - 8*A3*A4*A5^2 - 8*A3*A4*A6^2 + 16*A3*A4*A6*A8 + 32*A3*A4*A7^2 + 24*A3*A4*A8^2 - 24*A3*A4*l1^2 + 24*A4^2*A5^2*x12*y23 + 24*A4^2*A5^2*x12 - 8*A4^2*A5^2*x23*y23 - 8*A4^2*A5^2*x23 + 8*A4^2*A6*A7 + 24*A4^2*A7*A8 - 24*A5^4*x12^3*y23 - 24*A5^4*x12^3 + 24*A5^4*x12^2*x23*y23 + 24*A5^4*x12^2*x23 + 24*A5^4*x12*x23^2*y23 + 24*A5^4*x12*x23^2 - 32*A5^4*x12*y23^3 - 96*A5^4*x12*y23^2 - 72*A5^4*x12*y23 - 8*A5^4*x12 - 24*A5^4*x23^3*y23 - 24*A5^4*x23^3 + 32*A5^4*x23*y23^3 + 96*A5^4*x23*y23^2 + 88*A5^4*x23*y23 + 24*A5^4*x23 + 8*A5^2*A6^2*x12*y23 + 8*A5^2*A6^2*x12 - 24*A5^2*A6^2*x23*y23 - 24*A5^2*A6^2*x23 + 8*A5^2*A6*A7*x12^2 + 16*A5^2*A6*A7*x12*x23 - 24*A5^2*A6*A7*x23^2 + 32*A5^2*A6*A7*y23^2 + 64*A5^2*A6*A7*y23 + 24*A5^2*A6*A7 - 16*A5^2*A6*A8*x12*y23 - 16*A5^2*A6*A8*x12 - 16*A5^2*A6*A8*x23*y23 - 16*A5^2*A6*A8*x23 - 32*A5^2*A7^2*x12*y23 - 32*A5^2*A7^2*x12 + 32*A5^2*A7^2*x23*y23 + 32*A5^2*A7^2*x23 + 24*A5^2*A7*A8*x12^2 - 16*A5^2*A7*A8*x12*x23 - 8*A5^2*A7*A8*x23^2 + 32*A5^2*A7*A8*y23^2 + 64*A5^2*A7*A8*y23 + 8*A5^2*A7*A8 - 24*A5^2*A8^2*x12*y23 - 24*A5^2*A8^2*x12 + 8*A5^2*A8^2*x23*y23 + 8*A5^2*A8^2*x23 - 24*A5^2*l1^2*x12*y23 - 24*A5^2*l1^2*x12 + 8*A5^2*l1^2*x23*y23 + 8*A5^2*l1^2*x23 - 24*A6^3*A7 - 24*A6^2*A7*A8 + 32*A6*A7^3 + 24*A6*A7*A8^2 - 8*A6*A7*l1^2 + 32*A7^3*A8 + 24*A7*A8^3 - 24*A7*A8*l1^2);
            C7 = (4*A1^4*x12^4 - 8*A1^4*x12^3*x23 + 24*A1^4*x12^2*y23^2 + 48*A1^4*x12^2*y23 + 16*A1^4*x12^2 + 8*A1^4*x12*x23^3 - 48*A1^4*x12*x23*y23^2 - 96*A1^4*x12*x23*y23 - 40*A1^4*x12*x23 - 4*A1^4*x23^4 + 24*A1^4*x23^2*y23^2 + 48*A1^4*x23^2*y23 + 24*A1^4*x23^2 - 8*A1^4*y23^2 - 16*A1^4*y23 - 4*A1^4 + 8*A1^2*A2^2*x12*x23 - 8*A1^2*A2^2*x23^2 + 8*A1^2*A2^2*y23^2 + 16*A1^2*A2^2*y23 + 8*A1^2*A2^2 - 32*A1^2*A2*A3*x12*y23 - 32*A1^2*A2*A3*x12 + 32*A1^2*A2*A3*x23*y23 + 32*A1^2*A2*A3*x23 + 8*A1^2*A2*A4*x12^2 - 8*A1^2*A2*A4*x23^2 + 16*A1^2*A2*A4*y23^2 + 32*A1^2*A2*A4*y23 + 8*A1^2*A2*A4 + 8*A1^2*A3^2*x12^2 - 16*A1^2*A3^2*x12*x23 + 8*A1^2*A3^2*x23^2 - 8*A1^2*A3^2 - 32*A1^2*A3*A4*x12*y23 - 32*A1^2*A3*A4*x12 + 32*A1^2*A3*A4*x23*y23 + 32*A1^2*A3*A4*x23 + 8*A1^2*A4^2*x12^2 - 8*A1^2*A4^2*x12*x23 + 8*A1^2*A4^2*y23^2 + 16*A1^2*A4^2*y23 + 8*A1^2*A5^2*x12^4 - 16*A1^2*A5^2*x12^3*x23 + 48*A1^2*A5^2*x12^2*y23^2 + 96*A1^2*A5^2*x12^2*y23 + 32*A1^2*A5^2*x12^2 + 16*A1^2*A5^2*x12*x23^3 - 96*A1^2*A5^2*x12*x23*y23^2 - 192*A1^2*A5^2*x12*x23*y23 - 80*A1^2*A5^2*x12*x23 - 8*A1^2*A5^2*x23^4 + 48*A1^2*A5^2*x23^2*y23^2 + 96*A1^2*A5^2*x23^2*y23 + 48*A1^2*A5^2*x23^2 - 16*A1^2*A5^2*y23^2 - 32*A1^2*A5^2*y23 - 8*A1^2*A5^2 - 8*A1^2*A6^2*x12*x23 + 8*A1^2*A6^2*x23^2 - 8*A1^2*A6^2*y23^2 - 16*A1^2*A6^2*y23 - 8*A1^2*A6^2 + 32*A1^2*A6*A7*x12*y23 + 32*A1^2*A6*A7*x12 - 32*A1^2*A6*A7*x23*y23 - 32*A1^2*A6*A7*x23 - 8*A1^2*A6*A8*x12^2 + 8*A1^2*A6*A8*x23^2 - 16*A1^2*A6*A8*y23^2 - 32*A1^2*A6*A8*y23 - 8*A1^2*A6*A8 - 8*A1^2*A7^2*x12^2 + 16*A1^2*A7^2*x12*x23 - 8*A1^2*A7^2*x23^2 + 8*A1^2*A7^2 + 32*A1^2*A7*A8*x12*y23 + 32*A1^2*A7*A8*x12 - 32*A1^2*A7*A8*x23*y23 - 32*A1^2*A7*A8*x23 - 8*A1^2*A8^2*x12^2 + 8*A1^2*A8^2*x12*x23 - 8*A1^2*A8^2*y23^2 - 16*A1^2*A8^2*y23 + 8*A1^2*l1^2*x12^2 - 8*A1^2*l1^2*x12*x23 + 8*A1^2*l1^2*y23^2 + 16*A1^2*l1^2*y23 + 32*A1*A2*A5*A6*x12*x23 - 32*A1*A2*A5*A6*x23^2 + 32*A1*A2*A5*A6*y23^2 + 64*A1*A2*A5*A6*y23 + 32*A1*A2*A5*A6 - 64*A1*A2*A5*A7*x12*y23 - 64*A1*A2*A5*A7*x12 + 64*A1*A2*A5*A7*x23*y23 + 64*A1*A2*A5*A7*x23 + 16*A1*A2*A5*A8*x12^2 - 16*A1*A2*A5*A8*x23^2 + 32*A1*A2*A5*A8*y23^2 + 64*A1*A2*A5*A8*y23 + 16*A1*A2*A5*A8 - 64*A1*A3*A5*A6*x12*y23 - 64*A1*A3*A5*A6*x12 + 64*A1*A3*A5*A6*x23*y23 + 64*A1*A3*A5*A6*x23 + 32*A1*A3*A5*A7*x12^2 - 64*A1*A3*A5*A7*x12*x23 + 32*A1*A3*A5*A7*x23^2 - 32*A1*A3*A5*A7 - 64*A1*A3*A5*A8*x12*y23 - 64*A1*A3*A5*A8*x12 + 64*A1*A3*A5*A8*x23*y23 + 64*A1*A3*A5*A8*x23 + 16*A1*A4*A5*A6*x12^2 - 16*A1*A4*A5*A6*x23^2 + 32*A1*A4*A5*A6*y23^2 + 64*A1*A4*A5*A6*y23 + 16*A1*A4*A5*A6 - 64*A1*A4*A5*A7*x12*y23 - 64*A1*A4*A5*A7*x12 + 64*A1*A4*A5*A7*x23*y23 + 64*A1*A4*A5*A7*x23 + 32*A1*A4*A5*A8*x12^2 - 32*A1*A4*A5*A8*x12*x23 + 32*A1*A4*A5*A8*y23^2 + 64*A1*A4*A5*A8*y23 - 4*A2^4 - 8*A2^3*A4 + 24*A2^2*A3^2 - 8*A2^2*A5^2*x12*x23 + 8*A2^2*A5^2*x23^2 - 8*A2^2*A5^2*y23^2 - 16*A2^2*A5^2*y23 - 8*A2^2*A5^2 - 8*A2^2*A6^2 - 8*A2^2*A6*A8 + 8*A2^2*A7^2 + 48*A2*A3^2*A4 + 32*A2*A3*A5^2*x12*y23 + 32*A2*A3*A5^2*x12 - 32*A2*A3*A5^2*x23*y23 - 32*A2*A3*A5^2*x23 + 32*A2*A3*A6*A7 + 32*A2*A3*A7*A8 + 8*A2*A4^3 - 8*A2*A4*A5^2*x12^2 + 8*A2*A4*A5^2*x23^2 - 16*A2*A4*A5^2*y23^2 - 32*A2*A4*A5^2*y23 - 8*A2*A4*A5^2 - 8*A2*A4*A6^2 + 16*A2*A4*A7^2 + 8*A2*A4*A8^2 - 8*A2*A4*l1^2 + 24*A3^2*A4^2 - 8*A3^2*A5^2*x12^2 + 16*A3^2*A5^2*x12*x23 - 8*A3^2*A5^2*x23^2 + 8*A3^2*A5^2 + 8*A3^2*A6^2 + 16*A3^2*A6*A8 + 8*A3^2*A8^2 - 8*A3^2*l1^2 + 32*A3*A4*A5^2*x12*y23 + 32*A3*A4*A5^2*x12 - 32*A3*A4*A5^2*x23*y23 - 32*A3*A4*A5^2*x23 + 32*A3*A4*A6*A7 + 32*A3*A4*A7*A8 + 4*A4^4 - 8*A4^2*A5^2*x12^2 + 8*A4^2*A5^2*x12*x23 - 8*A4^2*A5^2*y23^2 - 16*A4^2*A5^2*y23 + 8*A4^2*A6*A8 + 8*A4^2*A7^2 + 8*A4^2*A8^2 - 8*A4^2*l1^2 + 4*A5^4*x12^4 - 8*A5^4*x12^3*x23 + 24*A5^4*x12^2*y23^2 + 48*A5^4*x12^2*y23 + 16*A5^4*x12^2 + 8*A5^4*x12*x23^3 - 48*A5^4*x12*x23*y23^2 - 96*A5^4*x12*x23*y23 - 40*A5^4*x12*x23 - 4*A5^4*x23^4 + 24*A5^4*x23^2*y23^2 + 48*A5^4*x23^2*y23 + 24*A5^4*x23^2 - 8*A5^4*y23^2 - 16*A5^4*y23 - 4*A5^4 + 8*A5^2*A6^2*x12*x23 - 8*A5^2*A6^2*x23^2 + 8*A5^2*A6^2*y23^2 + 16*A5^2*A6^2*y23 + 8*A5^2*A6^2 - 32*A5^2*A6*A7*x12*y23 - 32*A5^2*A6*A7*x12 + 32*A5^2*A6*A7*x23*y23 + 32*A5^2*A6*A7*x23 + 8*A5^2*A6*A8*x12^2 - 8*A5^2*A6*A8*x23^2 + 16*A5^2*A6*A8*y23^2 + 32*A5^2*A6*A8*y23 + 8*A5^2*A6*A8 + 8*A5^2*A7^2*x12^2 - 16*A5^2*A7^2*x12*x23 + 8*A5^2*A7^2*x23^2 - 8*A5^2*A7^2 - 32*A5^2*A7*A8*x12*y23 - 32*A5^2*A7*A8*x12 + 32*A5^2*A7*A8*x23*y23 + 32*A5^2*A7*A8*x23 + 8*A5^2*A8^2*x12^2 - 8*A5^2*A8^2*x12*x23 + 8*A5^2*A8^2*y23^2 + 16*A5^2*A8^2*y23 + 8*A5^2*l1^2*x12^2 - 8*A5^2*l1^2*x12*x23 + 8*A5^2*l1^2*y23^2 + 16*A5^2*l1^2*y23 - 4*A6^4 - 8*A6^3*A8 + 24*A6^2*A7^2 + 48*A6*A7^2*A8 + 8*A6*A8^3 - 8*A6*A8*l1^2 + 24*A7^2*A8^2 - 8*A7^2*l1^2 + 4*A8^4 - 8*A8^2*l1^2 + 4*l1^4);
            C8 = (8*A2^3*A3 + 8*A3*A4^3 + 8*A6^3*A7 + 8*A7*A8^3 + 8*A1^4*x12 + 8*A5^4*x12 - 8*A1^4*x23 - 8*A5^4*x23 - 8*A1^4*x12^3 - 8*A5^4*x12^3 + 8*A1^4*x23^3 + 8*A5^4*x23^3 - 8*A1^2*A2^2*x12 - 8*A1^2*A4^2*x12 + 16*A1^2*A5^2*x12 + 8*A1^2*A6^2*x12 + 8*A2^2*A5^2*x12 + 8*A1^2*A8^2*x12 + 8*A4^2*A5^2*x12 - 8*A5^2*A6^2*x12 - 8*A5^2*A8^2*x12 + 8*A1^2*A2^2*x23 + 8*A1^2*A4^2*x23 - 16*A1^2*A5^2*x23 - 8*A1^2*A6^2*x23 - 8*A2^2*A5^2*x23 - 8*A1^2*A8^2*x23 - 8*A4^2*A5^2*x23 + 8*A5^2*A6^2*x23 + 8*A5^2*A8^2*x23 - 8*A1^2*l1^2*x12 - 8*A5^2*l1^2*x12 + 8*A1^2*l1^2*x23 + 8*A5^2*l1^2*x23 - 24*A1^4*x12*x23^2 + 24*A1^4*x12^2*x23 - 24*A5^4*x12*x23^2 + 24*A5^4*x12^2*x23 - 8*A1^4*x12^3*y23 - 8*A5^4*x12^3*y23 + 8*A1^4*x23^3*y23 + 8*A5^4*x23^3*y23 - 16*A1^2*A5^2*x12^3 + 16*A1^2*A5^2*x23^3 - 8*A1^2*A2*A3 - 8*A1^2*A3*A4 + 24*A2*A3*A4^2 + 24*A2^2*A3*A4 + 8*A2*A3*A5^2 + 8*A2*A3*A6^2 + 8*A3*A4*A5^2 + 8*A2*A3*A8^2 + 8*A3*A4*A6^2 + 8*A1^2*A6*A7 + 8*A3*A4*A8^2 + 8*A2^2*A6*A7 + 8*A1^2*A7*A8 + 8*A2^2*A7*A8 + 8*A4^2*A6*A7 - 8*A5^2*A6*A7 + 8*A4^2*A7*A8 - 8*A5^2*A7*A8 + 24*A6*A7*A8^2 + 24*A6^2*A7*A8 - 8*A2*A3*l1^2 - 8*A3*A4*l1^2 - 8*A6*A7*l1^2 - 8*A7*A8*l1^2 + 8*A1^4*x12*y23 + 8*A5^4*x12*y23 - 8*A1^4*x23*y23 - 8*A5^4*x23*y23 + 8*A1^2*A2*A3*x12^2 + 8*A1^2*A3*A4*x12^2 - 8*A2*A3*A5^2*x12^2 - 8*A3*A4*A5^2*x12^2 - 8*A1^2*A6*A7*x12^2 - 8*A1^2*A7*A8*x12^2 + 8*A1^2*A2*A3*x23^2 + 8*A5^2*A6*A7*x12^2 + 8*A1^2*A3*A4*x23^2 + 8*A5^2*A7*A8*x12^2 - 8*A2*A3*A5^2*x23^2 - 8*A3*A4*A5^2*x23^2 - 8*A1^2*A6*A7*x23^2 - 8*A1^2*A7*A8*x23^2 + 8*A5^2*A6*A7*x23^2 + 8*A5^2*A7*A8*x23^2 - 16*A1*A2*A5*A7 - 16*A1*A3*A5*A6 - 16*A1*A3*A5*A8 - 16*A1*A4*A5*A7 + 16*A2*A3*A6*A8 + 16*A2*A4*A6*A7 + 16*A2*A4*A7*A8 + 16*A3*A4*A6*A8 - 8*A1^2*A2^2*x12*y23 - 8*A1^2*A4^2*x12*y23 + 16*A1^2*A5^2*x12*y23 + 8*A1^2*A6^2*x12*y23 + 8*A2^2*A5^2*x12*y23 + 8*A1^2*A8^2*x12*y23 + 8*A4^2*A5^2*x12*y23 - 8*A5^2*A6^2*x12*y23 - 8*A5^2*A8^2*x12*y23 + 8*A1^2*A2^2*x23*y23 + 8*A1^2*A4^2*x23*y23 - 16*A1^2*A5^2*x23*y23 - 8*A1^2*A6^2*x23*y23 - 8*A2^2*A5^2*x23*y23 - 8*A1^2*A8^2*x23*y23 - 8*A4^2*A5^2*x23*y23 + 8*A5^2*A6^2*x23*y23 + 8*A5^2*A8^2*x23*y23 - 8*A1^2*l1^2*x12*y23 - 8*A5^2*l1^2*x12*y23 + 8*A1^2*l1^2*x23*y23 + 8*A5^2*l1^2*x23*y23 - 24*A1^4*x12*x23^2*y23 + 24*A1^4*x12^2*x23*y23 - 24*A5^4*x12*x23^2*y23 + 24*A5^4*x12^2*x23*y23 - 48*A1^2*A5^2*x12*x23^2 + 48*A1^2*A5^2*x12^2*x23 - 16*A1^2*A5^2*x12^3*y23 + 16*A1^2*A5^2*x23^3*y23 - 16*A1^2*A2*A4*x12 + 16*A2*A4*A5^2*x12 + 16*A1^2*A6*A8*x12 + 16*A1^2*A2*A4*x23 - 16*A5^2*A6*A8*x12 - 16*A2*A4*A5^2*x23 - 16*A1^2*A6*A8*x23 + 16*A5^2*A6*A8*x23 + 16*A1*A2*A5*A7*x12^2 + 16*A1*A3*A5*A6*x12^2 + 16*A1*A3*A5*A8*x12^2 + 16*A1*A4*A5*A7*x12^2 + 16*A1*A2*A5*A7*x23^2 + 16*A1*A3*A5*A6*x23^2 + 16*A1*A3*A5*A8*x23^2 + 16*A1*A4*A5*A7*x23^2 - 48*A1^2*A5^2*x12*x23^2*y23 + 48*A1^2*A5^2*x12^2*x23*y23 - 16*A1^2*A2*A3*x12*x23 - 16*A1^2*A3*A4*x12*x23 + 16*A2*A3*A5^2*x12*x23 + 16*A3*A4*A5^2*x12*x23 + 16*A1^2*A6*A7*x12*x23 + 16*A1^2*A7*A8*x12*x23 - 16*A5^2*A6*A7*x12*x23 - 16*A5^2*A7*A8*x12*x23 - 16*A1^2*A2*A4*x12*y23 + 16*A2*A4*A5^2*x12*y23 + 16*A1^2*A6*A8*x12*y23 + 16*A1^2*A2*A4*x23*y23 - 16*A5^2*A6*A8*x12*y23 - 16*A2*A4*A5^2*x23*y23 - 16*A1^2*A6*A8*x23*y23 + 16*A5^2*A6*A8*x23*y23 - 32*A1*A2*A5*A6*x12 - 32*A1*A2*A5*A8*x12 - 32*A1*A4*A5*A6*x12 - 32*A1*A4*A5*A8*x12 + 32*A1*A2*A5*A6*x23 + 32*A1*A2*A5*A8*x23 + 32*A1*A4*A5*A6*x23 + 32*A1*A4*A5*A8*x23 - 32*A1*A2*A5*A7*x12*x23 - 32*A1*A3*A5*A6*x12*x23 - 32*A1*A3*A5*A8*x12*x23 - 32*A1*A4*A5*A7*x12*x23 - 32*A1*A2*A5*A6*x12*y23 - 32*A1*A2*A5*A8*x12*y23 - 32*A1*A4*A5*A6*x12*y23 - 32*A1*A4*A5*A8*x12*y23 + 32*A1*A2*A5*A6*x23*y23 + 32*A1*A2*A5*A8*x23*y23 + 32*A1*A4*A5*A6*x23*y23 + 32*A1*A4*A5*A8*x23*y23);
            C9 = (A1^4*x12^4 - 4*A1^4*x12^3*x23 + 6*A1^4*x12^2*x23^2 - 2*A1^4*x12^2 - 4*A1^4*x12*x23^3 + 4*A1^4*x12*x23 + A1^4*x23^4 - 2*A1^4*x23^2 + A1^4 + 2*A1^2*A2^2*x12^2 - 4*A1^2*A2^2*x12*x23 + 2*A1^2*A2^2*x23^2 - 2*A1^2*A2^2 + 4*A1^2*A2*A4*x12^2 - 8*A1^2*A2*A4*x12*x23 + 4*A1^2*A2*A4*x23^2 - 4*A1^2*A2*A4 + 2*A1^2*A4^2*x12^2 - 4*A1^2*A4^2*x12*x23 + 2*A1^2*A4^2*x23^2 - 2*A1^2*A4^2 + 2*A1^2*A5^2*x12^4 - 8*A1^2*A5^2*x12^3*x23 + 12*A1^2*A5^2*x12^2*x23^2 - 4*A1^2*A5^2*x12^2 - 8*A1^2*A5^2*x12*x23^3 + 8*A1^2*A5^2*x12*x23 + 2*A1^2*A5^2*x23^4 - 4*A1^2*A5^2*x23^2 + 2*A1^2*A5^2 - 2*A1^2*A6^2*x12^2 + 4*A1^2*A6^2*x12*x23 - 2*A1^2*A6^2*x23^2 + 2*A1^2*A6^2 - 4*A1^2*A6*A8*x12^2 + 8*A1^2*A6*A8*x12*x23 - 4*A1^2*A6*A8*x23^2 + 4*A1^2*A6*A8 - 2*A1^2*A8^2*x12^2 + 4*A1^2*A8^2*x12*x23 - 2*A1^2*A8^2*x23^2 + 2*A1^2*A8^2 + 2*A1^2*l1^2*x12^2 - 4*A1^2*l1^2*x12*x23 + 2*A1^2*l1^2*x23^2 - 2*A1^2*l1^2 + 8*A1*A2*A5*A6*x12^2 - 16*A1*A2*A5*A6*x12*x23 + 8*A1*A2*A5*A6*x23^2 - 8*A1*A2*A5*A6 + 8*A1*A2*A5*A8*x12^2 - 16*A1*A2*A5*A8*x12*x23 + 8*A1*A2*A5*A8*x23^2 - 8*A1*A2*A5*A8 + 8*A1*A4*A5*A6*x12^2 - 16*A1*A4*A5*A6*x12*x23 + 8*A1*A4*A5*A6*x23^2 - 8*A1*A4*A5*A6 + 8*A1*A4*A5*A8*x12^2 - 16*A1*A4*A5*A8*x12*x23 + 8*A1*A4*A5*A8*x23^2 - 8*A1*A4*A5*A8 + A2^4 + 4*A2^3*A4 + 6*A2^2*A4^2 - 2*A2^2*A5^2*x12^2 + 4*A2^2*A5^2*x12*x23 - 2*A2^2*A5^2*x23^2 + 2*A2^2*A5^2 + 2*A2^2*A6^2 + 4*A2^2*A6*A8 + 2*A2^2*A8^2 - 2*A2^2*l1^2 + 4*A2*A4^3 - 4*A2*A4*A5^2*x12^2 + 8*A2*A4*A5^2*x12*x23 - 4*A2*A4*A5^2*x23^2 + 4*A2*A4*A5^2 + 4*A2*A4*A6^2 + 8*A2*A4*A6*A8 + 4*A2*A4*A8^2 - 4*A2*A4*l1^2 + A4^4 - 2*A4^2*A5^2*x12^2 + 4*A4^2*A5^2*x12*x23 - 2*A4^2*A5^2*x23^2 + 2*A4^2*A5^2 + 2*A4^2*A6^2 + 4*A4^2*A6*A8 + 2*A4^2*A8^2 - 2*A4^2*l1^2 + A5^4*x12^4 - 4*A5^4*x12^3*x23 + 6*A5^4*x12^2*x23^2 - 2*A5^4*x12^2 - 4*A5^4*x12*x23^3 + 4*A5^4*x12*x23 + A5^4*x23^4 - 2*A5^4*x23^2 + A5^4 + 2*A5^2*A6^2*x12^2 - 4*A5^2*A6^2*x12*x23 + 2*A5^2*A6^2*x23^2 - 2*A5^2*A6^2 + 4*A5^2*A6*A8*x12^2 - 8*A5^2*A6*A8*x12*x23 + 4*A5^2*A6*A8*x23^2 - 4*A5^2*A6*A8 + 2*A5^2*A8^2*x12^2 - 4*A5^2*A8^2*x12*x23 + 2*A5^2*A8^2*x23^2 - 2*A5^2*A8^2 + 2*A5^2*l1^2*x12^2 - 4*A5^2*l1^2*x12*x23 + 2*A5^2*l1^2*x23^2 - 2*A5^2*l1^2 + A6^4 + 4*A6^3*A8 + 6*A6^2*A8^2 - 2*A6^2*l1^2 + 4*A6*A8^3 - 4*A6*A8*l1^2 + A8^4 - 2*A8^2*l1^2 + l1^4);

            % the coefficient matrix of the 8-degree polynomials equation
            CoefficientMatrix8_Sym = [C1, C2, C3, C4, C5, C6, C7, C8, C9];
            
            % calculate the root of the 8-degree polynomials equation
            x_Sym = roots(CoefficientMatrix8_Sym);
            %%----------------------------------------------------------------------------
                        
            % get the value of real number (+/-)
            x = x_Sym(imag(x_Sym) == 0);
            NumRealq13 = length (x);   
            
            % the final value of "+/-theta13"
            % the programm should judge the sign of the value            
            j =0;
            for i = 1:length (x)
                if  2 * atan(x(i)) > 0
                    j = j + 1;
                    % x = tan(q22/2)
                    q22all(j) =  2 * atan(x(i));
                else
                    continue;
                end
            end
            %% ------------------Obtain all of the correct values and assign to q13q23-------------------------
            % numbers of i and j are used to count the possible values that satisfy the
            % condiation of C1z = C2z; And then, assign all the possible values to
            % matrix q13q23;
            
            %tic
            for Numq22 = 1:length(q22all)
                
                q22SingleValue = q22all(Numq22);
                
              %% -------------- Calculate the value of q23 (theta23) ----------------
                % (q22+q23all)*180/pi
                % sin(q12) + sin(q12+q13SingleValue) - sin(q22) - sin(q22+q23all) == 0
                %  For real elements of x in the interval [-1,1], asin(x) returns values in the interval [-pi/2,pi/2]
                % iterative method to get the optimal value
                %
                j = 0;
                delta_q22 = 0.1;
                while(abs(sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12)) >= 1 && j <= 500)
                    j = j + 1;
                    if q22SingleValue + q23 > pi/2 || (q22SingleValue + q23 > -pi/2 && q22SingleValue + q23 <= 0)
                        q22SingleValue = q22SingleValue + delta_q22;
                    elseif q22SingleValue + q23 < -pi/2 || (q22SingleValue + q23 > 0 && q22SingleValue + q23 < pi/2)
                        q22SingleValue = q22SingleValue - delta_q22;
                    end
                end
                %
                
                q13all(1) = asin(sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12)) - q12;
                if sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12) <= 0
                    q13all(2) =  - pi - asin(sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12)) - q12;
                else
                    q13all(2) =    pi - asin(sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12)) - q12;
                end
                
                for Numq13 = 1:length(q13all)
                    q13SingleValue = q13all(Numq13); %1.200958270516672 % q13SingleValue = 1.2373
                    %%-----------------------Get the output values of Moving Platform-----------------------
                    %%--------------------Calculate the position of Ai Bi Ci------------------
                    A1 = [0, -L1/2, 0];
                    B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                    
                    A2 = [0, L1/2, 0];
                    B2 = [L2 * cos(q22SingleValue) * sin(q21), L1/2 - L2 * cos(q22SingleValue) * cos(q21), L2 * sin(q22SingleValue)];
                    C2 = [L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * sin(q21), L1/2 - L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * cos(q21),...
                        L2 * (sin(q22SingleValue) + sin(q22SingleValue + q23))];
                    %%------------------------------------------------------------------------
                    JudgeLength_C1C2(Numq13) = abs(norm(C1 - C2) - L1);
                    if abs(norm(C1 - C2) - L1) < 1e-12
                        %Plot;
                        break
                    end                    
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
                            abs(abs( q13all(col(1)) ) - pi) > 1e-8 && abs(JudgeLength_C1C2(col(1))) < 1e-8
                        JudgeLength_C1C2_min = JudgeLength_C1C2(col(1));
                    else
                        %---------- Method II ----------
                        % Iterative Method so as to approcimate the required value
                        for OnlyUsedForFolding_CircleOptimalPick = 1:1
                            % We simple the one cycle points
                            ii = 0;
                            for i_angle = 0:1*pi/180:pi
                                q22SingleValue = i_angle;
                                %
                                sin_value = sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12);
                                zC2 = sin(q22SingleValue) + sin(q22SingleValue + q23);
                                %
                                if sin_value > 1 || zC2 < 0
                                    continue;
                                end
                                ii = ii + 1;
                                q22SingleValue_mat(ii) = i_angle;
                                %
                                q13all(1) = asin(sin_value) - q12;
                                if sin_value <= 0
                                    q13all(2) =  - pi - asin(sin_value) - q12;
                                else
                                    q13all(2) =    pi - asin(sin_value) - q12;
                                end
                                for i_q13all = 1:2
                                    q13SingleValue = q13all(i_q13all);
                                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                    C2 = [L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * sin(q21), L1/2 - L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * cos(q21),...
                                        L2 * (sin(q22SingleValue) + sin(q22SingleValue + q23))];
                                    JudgeLength_C1C2_Solution(i_q13all,ii) = norm(C1 - C2) - L1;
                                    q13SingleValue_mat(i_q13all,ii) = q13all(i_q13all);
                                    if i_q13all == 1
                                        C11_mat(ii,:) = C1;
                                        C21_mat(ii,:) = C2;
                                    else
                                        C12_mat(ii,:) = C1;
                                        C22_mat(ii,:) = C2;
                                    end
                                    
                                end
                            end
                            plot3(C11_mat(:,1), C11_mat(:,2), C11_mat(:,3),'k-.','linewidth',1); hold on;
                            plot3(C21_mat(:,1), C21_mat(:,2), C21_mat(:,3),'k-.','linewidth',1); hold on;
                            plot3(C12_mat(:,1), C12_mat(:,2), C12_mat(:,3),'k-','linewidth',1); hold on;
                            plot3(C22_mat(:,1), C22_mat(:,2), C22_mat(:,3),'k-','linewidth',1); hold on; 
                            axis equal;
                            grid on;
                            
                            JudgeLength_C1C2_min = [];
                            for i_q13all = 1:2
                                B = JudgeLength_C1C2_Solution(i_q13all,:);
                                col_B_positive = find(B>0);
                                col_B_zero = find(abs(B) <= 1e-8);
                                if isempty(col_B_zero) ~= 1 && min(col_B_zero) < length(B)
                                    Changepoint_positive_min = min(col_B_zero);
                                    i_q13all_selected = i_q13all;
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
                                        i_q13all_selected = i_q13all;
                                    end
                                end
                            end  
                                q22SingleValue = q22SingleValue_mat(Changepoint_positive_min);
                                q13SingleValue = q13SingleValue_mat(i_q13all_selected,Changepoint_positive_min);
                                C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                    L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                C2 = [L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * sin(q21), L1/2 - L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * cos(q21),...
                                    L2 * (sin(q22SingleValue) + sin(q22SingleValue + q23))];
                                
                                plot3(C12_mat(Changepoint_positive_min,1), C12_mat(Changepoint_positive_min,2), C12_mat(Changepoint_positive_min,3),'r.','markersize',10); hold on;
                                
                                %---------------------- Iterative Start --------------------
                                delta_q22 = 0.05;
                                q22SingleValue = q22SingleValue - delta_q22;
                                % norm(C1 - C2) - L1
                                SignChange_1 = 0;
                                SignChange_2 = 0;
                                k = 0;
                                % iterative method to get the optimal value
                                while(abs(norm(C1 - C2) - L1) > 1e-10 && k <= 500)
                                    k = k + 1;
                                    % Here, we must make sure 'sin_value belongs to [-1 1]'
                                    sin_value = sin(q22SingleValue) + sin(q22SingleValue + q23) - sin(q12);
                                    %
                                    q13all(1) = asin(sin_value) - q12;
                                    if sin_value <= 0
                                        q13all(2) =  - pi - asin(sin_value) - q12;
                                    else
                                        q13all(2) =    pi - asin(sin_value) - q12;
                                    end
                                    
                                    % Choose the correct q13all
                                    q13SingleValue = q13all(i_q13all_selected);
                                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                                    C2 = [L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * sin(q21), L1/2 - L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * cos(q21),...
                                        L2 * (sin(q22SingleValue) + sin(q22SingleValue + q23))];
                                    
                                    JudgeLength_C1C2_min = norm(C1 - C2) - L1;
                                    %JudgeLength_C1C2_min
                                    if JudgeLength_C1C2_min > 0
                                        if SignChange_1 == 0
                                            if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min < 0
                                                delta_q22 = 0.5 * delta_q22;
                                            elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min >= 0
                                                delta_q22 = - 0.5 * delta_q22;
                                            end
                                        end
                                        SignChange_1 = SignChange_1 + 1;
                                        SignChange_2 = 0;
                                    elseif JudgeLength_C1C2_min < 0
                                        if SignChange_2 == 0
                                            if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min > 0
                                                delta_q22 = 0.5 * delta_q22;
                                            elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min <= 0
                                                delta_q22 = - 0.5 * delta_q22;
                                            end
                                        end
                                        SignChange_1 = 0;
                                        SignChange_2 = SignChange_2 + 1;
                                    end
                                    
                                    q22SingleValue = q22SingleValue - delta_q22;
                                    %hold on
                                    %Plot;
                                end
                                
                                %---------------------- Iterative end --------------------
                        end
                    end
                    %--------------------------------------------------------------------
                    
                    % assign the values to q13q23
                    q22 = q22SingleValue;
                    q13 = q13SingleValue;
                    
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
                    q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
                else
                    display('No solution for this input, Calculation process is stopped');
                    p = [];
                    ABC = [];
                    q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
                end
                
            end
            %toc
            
          %% --------------------Plot the mechanism Ai Bi Ci------------------            
          % PA1B1C1x = [A1(1), B1(1), C1(1)];
          % PA1B1C1y = [A1(2), B1(2), C1(2)];
          % PA1B1C1z = [A1(3), B1(3), C1(3)];
          % plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
          %
          % PA2B2C2x = [A2(1), B2(1), C2(1)];
          % PA2B2C2y = [A2(2), B2(2), C2(2)];
          % PA2B2C2z = [A2(3), B2(3), C2(3)];
          % plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
          %
          % PC1C2x = [C1(1), C2(1)];
          % PC1C2y = [C1(2), C2(2)];
          % PC1C2z = [C1(3), C2(3)];
          % plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
          %
          % PA1A2x = [A1(1), A2(1)];
          % PA1A2y = [A1(2), A2(2)];
          % PA1A2z = [A1(3), A2(3)];
          % plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
          %
          % grid on;
          % axis equal;
          % xlabel('x');
          % ylabel('y');
          % zlabel('z');
          %
          % %----------------- plot xyz axes of base point --------------
          % x_axis = [0.1 0 0];
          % y_axis = [0 0.1 0];
          % z_axis = [0 0 0.1];
          % OP= [0 0 0];
          % xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
          % i = 1:2;
          % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
          % i = 3:4;
          % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
          % i = 5:6;
          % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
          % axis equal;
          %-----------------------------------------------------------
            
        end
               
    end
    
end

