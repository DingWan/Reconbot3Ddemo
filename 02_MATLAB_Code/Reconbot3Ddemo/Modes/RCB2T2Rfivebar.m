classdef RCB2T2Rfivebar
    % RCB2T2Rfivebar operational mode
    % RCB2T2Rfivebar is one mode with the motion of platfrom 2 Transition motion along xz-axis and 1 rotation around y-axis        
    %%------------2-RER 2T2Rfivebar inverse and Forward kinematics--------------
    %--------------------------------------------------------------------
    %     This function is to calculate the inverse kinematics of 4 DoF 2-RER PM
    %   mechanism with 2T1R, with y =0, q11 = q21 = +/- pi/2, case;
    %     The IK function: [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_1T3R_RotAroundPoint_IK(obj)   
    %     This is written by Wan Ding in 13 Dec 2016.
    %---------------------------------------------------------------------
    
    properties
        l1;
        l2;
        pos;
        q11q12q14q22;
    end
    
    methods
        function obj = RCB2T2Rfivebar(pos,q11q12q14q22,L1,L2)
            if nargin > 0
                obj.l1 = L1;
                obj.l2 = L2;
                obj.pos = pos;
                obj.q11q12q14q22 = q11q12q14q22;
            end
        end
        
        function [p, EulerAngle_q11_theta, ABC_FeasibleSolution, q1q2_FeasibleSolution, WSvalue] = RCB_2T2R_FiveBar_IK(obj)
            % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 0 1 1]
            % p = [x, 0, z, [], beta, 0]
            L1 = obj.l1;
            L2 = obj.l2;
            po = obj.pos;
            for i = 1:3
                if isempty(po{i}) == 0
                    p(i) = po{i};
                end
            end
            
            theta  = po{5};
            
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
            
            %% ---------- Calculate rotation matrix of Five-bar modes ----------
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
                WSvalue_2T2R_SinguPosA2C2 = 1;
            else
                % Euler angle IK
                    % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 0 1 1]
                    % p = [x, 0, z, 0, beta, 0]
                po{2} = 0;    
                q11 = pi/2; % inputs: beta, q11; 
                [EulerAngle_q11_theta] = EulerAngles_theta_q11_IK(theta, q11);
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
            
            po{4} = alpha;
            po{5} = beta;
            po{6} = gamma;
            name = '2T2R-FiveBar';
            fprintf('Mode %s inputs are: PosOri = [%.6g, %.6g, %.6g, %.6g, %.6g, %.6g].\n', ...
                        name, po{1}, po{2}, po{3}, po{4}*180/pi, po{5}*180/pi, po{6}*180/pi);
                    
            %% ------Calculate eight possbile outputs for ABC(1:8), q1q2(1:8)-----
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
            
             %% ----------------------- Calculate one solutions for one input  -----------------------
            if p(1) == 0 && p(2) == 0 && p(3) == 0
                WSvalue_2T2R_SinguPosA1C1 = 1;
                WSvalue_2T2R_SinguPosA2C2 = 1;
                IterationNumber = 1;
                q13 = pi;
                q14 = q12;
                q15 = q11;
                
                q23 = pi;
                q24 = q22;
                q25 = q21;
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
                        q24 = pi/2 - (q22 + q23 + theta);
                    else
                        q22 = angle_A2C2_k2 - angleB2A2C2;
                        q24 = pi/2 + theta - (q22 + q23);
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
                        if C1(jA1C1,:) - C1_in_Ob <= 1e-6
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
                        if C2(jA2C2,:) - C2_in_Ob <= 1e-6
                            q2(jA2C2,1:5) = q1q2(i,6:10);
                            A2B2C(jA2C2,:) = [A2(jA2C2,:), B2(jA2C2,:), C2(jA2C2,:)];
                        end
                    end
                end
                
                % Here, I did a small trick:
                % The number of correct value of q1 and q2 might be different,
                % so, I force the number to be the same by compensating the
                % fewer one with the missing number (jA1C1-jA2C2) of first value q1(1,1:5) and A1B1C1(1,:)
                Num_iijj = 0;
                if jA1C1 ~= 0 && jA2C2 ~= 0                    
                    for ii = 1:1:jA1C1
                        for jj = 1:1:jA2C2 
                            % Here you must calculate the x values of B1 and B2,
                            % compare it again, so as to make sure its three-bar not five-bar state
                            B1x = L2 * cos(q1(ii,2)) * sin(q1(ii,1));
                            B2x = L2 * cos(q2(jj,2)) * sin(q2(jj,1));                            
                            if abs(B1x - B2x) <= 1e-6
                                WSvalue_FiveBar = 0;                                 
                            else  
                                Num_iijj = Num_iijj + 1;
                                WSvalue_FiveBar = 1;
                                q1q2_FeasibleSolution(Num_iijj,:) = [q1(ii,1:5), q2(jj,1:5)];
                                ABC_FeasibleSolution(Num_iijj,:) = [A1B1C1(ii,:),A2B2C(jj,:)];
                            end
                        end 
                    end
                else
                    WSvalue_FiveBar = 0;
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                end
                if Num_iijj > 0
                    WSvalue_FiveBar = 1;
                else                    
                    q1q2_FeasibleSolution = [];
                    ABC_FeasibleSolution = [];
                end
                WSvalue_2T2R_SinguPosA1C1 = 0;
                WSvalue_2T2R_SinguPosA2C2 = 0;
            end
            WSvalue = [WSvalue_FiveBar, WSvalue_2T2R_SinguPosA1C1, WSvalue_2T2R_SinguPosA2C2];      
            p = [po{1}, po{2}, po{3}, EulerAngle];  
        end
        
        function [p, ABC, q1q2] = RCB_2T2R_FiveBar_FK(obj)
            %%------------Inputs-------------
            % Variable value assignment
            q11 = obj.q11q12q14q22(1);
            q12 = obj.q11q12q14q22(2);
            q14 = obj.q11q12q14q22(3);
            q22 = obj.q11q12q14q22(4);
            L1 = obj.l1;
            L2 = obj.l2;
            
            %% ----------------Basic calculaion of planar parallelgram five-bar linkage---------------
            q21 = - q11;
            %The output angle q13, q23, q24 can be calculated as follows:
            q13 = pi - q12 - q22;
            q23 = pi - q12 - q22;
            q24 = pi - (q12 + q13 + q14 + q22 + q23);%q22 - (q12 + q14);
            %--------------------- Parallelogram five-bar linkage ------------------
            
            %% -----------------------Get the output values of Moving Platform-----------------------
            %%--------------------Calculate the position of Ai Bi Ci------------------
            A1 = [0, -L1/2, 0];
            B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
            C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
            
            A2 = [0, L1/2, 0];
            B2 = [L2 * cos(q22) * sin(q21), L1/2 - L2 * cos(q22) * cos(q21), L2 * sin(q22)];
            C2 = [L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 - L2 * (cos(q22) + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
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
            % %----------------- plot xyz axes of base point --------------
            % x_axis = [0.5 0 0];
            % y_axis = [0 0.5 0];
            % z_axis = [0 0 0.5];
            % OP= [0 0 0];
            % xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
            % i = 1:2;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');hold on;
            % i = 3:4;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');hold on;
            % i = 5:6;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');hold on;
            % %-----------------------------------------------------------
            % %------------------plot xyz axes of Moving Platform----------------
            % xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(Matrix_from_axis_angle * transpose(xyz));
            % i = 1:2;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');hold on;
            % i = 3:4;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');hold on;
            % i = 5:6;
            % plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');hold on;
            % %----------------------------------------------
            % 
            % grid on;
            % xlabel('x');
            % ylabel('y');
            % zlabel('z');
            % axis equal;
            %%------------------------------------------------------------------------
        end
        
    end
    
end

