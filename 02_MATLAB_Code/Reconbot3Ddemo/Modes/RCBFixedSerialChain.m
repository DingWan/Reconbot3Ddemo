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
        
        function [p, EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(obj)
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
                case 6 % Serial A1C1 & A2C2
                    q11 = 0;
                    q12 = 0;
                    q21 = 0;
                    q22 = 0;
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
                %     %% --------------------Plot the mechanism Ai Bi Ci------------------
                %              PA1B1C1x = [A1(i,1), B1(i,1), C1(i,1)];
                %              PA1B1C1y = [A1(i,2), B1(i,2), C1(i,2)];
                %              PA1B1C1z = [A1(i,3), B1(i,3), C1(i,3)];
                %              plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
                %
                %              PA2B2C2x = [A2(i,1), B2(i,1), C2(i,1)];
                %              PA2B2C2y = [A2(i,2), B2(i,2), C2(i,2)];
                %              PA2B2C2z = [A2(i,3), B2(i,3), C2(i,3)];
                %              plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
                %
                %              PC1C2x = [C1(i,1), C2(i,1)];
                %              PC1C2y = [C1(i,2), C2(i,2)];
                %              PC1C2z = [C1(i,3), C2(i,3)];
                %              plot3(PC1C2x, PC1C2y, PC1C2z,'k-','linewidth',3); hold on;
                %
                %              PA1A2x = [A1(i,1), A2(i,1)];
                %              PA1A2y = [A1(i,2), A2(i,2)];
                %              PA1A2z = [A1(i,3), A2(i,3)];
                %              plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
                %
                %             %----------------- plot xyz axes of base point --------------
                %             x_axis = [0.5 0 0];
                %             y_axis = [0 0.5 0];
                %             z_axis = [0 0 0.5];
                %             OP= [0 0 0];
                %             xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
                %                 j = 1:2;
                %                 plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2); hold on
                %                 j = 3:4;
                %                 plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2); hold on
                %                 j = 5:6;
                %                 plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2); hold on
                %             %-----------------------------------------------------------
                %             %------------------plot xyz axes of Moving Platform----------------
                %             RotationMatrix_from_axis_angle1 = eul2rotm(EulerAngle,'ZYX');
                %             %RotationMatrix_from_axis_angle1 = eul2rotm(EulerAngle_q11_theta_2ndSolution(1:3),'ZYX');
                %             xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(RotationMatrix_from_axis_angle1 * transpose(xyz));
                %             j = 1:2;
                %             plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2);
                %             j = 3:4;
                %             plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2);
                %             j = 5:6;
                %             plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2);
                %             hold on;
                %             axis equal;
                %             xlabel('x');
                %             ylabel('y');
                %             zlabel('z');
                %             %----------------------------------------------
                hold off
                RCB_ABCplot3;
            end
            %------------------------------------------------------------------------
            p = [po{1}, po{2}, po{3}, EulerAngle];
        end
        
        function [p, ABC, q1q2] = RCB_FixedSerialChain_FK(obj)
            p = [];
            ABC = [];
            q1q2 = [];
        end
        
    end
    
end

