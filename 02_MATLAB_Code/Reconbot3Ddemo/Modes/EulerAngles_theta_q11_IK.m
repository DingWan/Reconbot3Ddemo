% 2-RER PM Euler angle inverse kinematics
% 

% In different operational modes, the inputs ([beta, gamma, q11]) for get the EulerAngle ([alpha, beta, gamma])
% will be different;
% Therefore, this function deals with this problems. 
% It can solve the modes: 1. General six-bar linkage (Input:[q11(x,y), gammma])
%                         2. Planar five-bar Parallel Linkage (Input:[q11(pi/2), beta])
%                         3. Planar Three-bar Serial Linkage (Input:[q11(x,y), gammma])
%                         4. Rotate around point p(1:3)  (Input:[beta, gammma])
%
%%-----------------2-RER PM Euler angle inverse kinematics--------------------
%-----------------------------------------------------------------------------
%     This function is to calculate the inverse Euler angle according to
%     the corresponding inputs ([beta, gamma, q11])
% 
%     The function: [EulerAngle_q11_theta] = EulerAngles_beta_gamma_q11_IK(beta, gamma, q11)
%
%     This is written by Wan Ding in 15 Nov 2016.
%     The copyright is belong to Wan Ding.
%
%       p                                  output position of moving platform (MP):[x, y, z]
%       q11                                first input angle of chain A1C1 (q11-q15)
%       theta                              input angle of MP (C1C2 and A1A2) in planer Six-bar linkage
%       EulerAngle_q11_theta               Euler Angle(Z-Y-X)  q11, and theta: [alpha, beta, gamma, q11, theta]
%       alpha                              rotation of the MP around Z-axis (Euler Angle, Z-Y-X)
%       beta                               rotation of the MP around Y-axis (Euler Angle, Z-Y-X)
%       gamma                              rotation of the MP around X-axis (Euler Angle, Z-Y-X)
%       ABC                                The position of six points A1B1C1,A2B2C2
%       q1q2                               The all angles q11-q15, q21-q25

%       abs_cos_theta                      absolute value of cos(theta)
%       abs_tan_beta                       absolute value of tan(beta)
%       cos_theta_2solutions               2 (+/-) solutions of cos(theta)
%       tan_beta_2solutions                2 (+/-) solutions of tan(beta)
%       theta_4solutions                   4 (+/-) solutions of theta
%       beta_4solutions                    4 (+/-) solutions of beta
%       alpha_4solutions                   4 (+/-) solutions of alpha
%       q11_4solutions                     4 (+/-) solutions of q11
%       IterationNumber                    iteration number given by the number of solutions, 4/8 

%       x                                  x = cos(q11);
%       y                                  y = cos(theta)
%       v                                  v = 1 + tan(beta)^2; 
%       w                                  w = tan(gamma)^2;

%       abs_cos_q11                        absolute value of cos(q11)
%       abs_tan_beta                       absolute value of tan(beta)
%       cosq11_2solutions                  2 (+/-) solutions of cos(q11)
%       costheta_1solutions                1 solution of cos(theta), because cos(theta)>0 as theta <-pi/2, pi/2>
%       q11_8solutions                     8 (+/-) solutions of q11
%       theta_8solutions                   4 (+/-) solutions of theta
%       alpha_8solutions                   4 (+/-) solutions of alpha

%       u_RotationAxis                     rotation axis for rotation matrix:u_RotationAxis = [cos(q11), sin(q11), 0];
%       r                                  rotation vector for rotation matrix: r = [u_RotationAxis, theta]; 
%       RotationMatrix_from_axis_angle     rotation matrix with rotation axis and angle as inputs:
%       eul_alpha_beta_gamma               Three Euler angles with Z-Y-X
%       m                                  Record the number of existed solutions
%       k                                  for loop 


%% q11 (x) and gamma (w,(tan(gamma)^2)) as inputs to calculate the output cos_q11 (x) and beta (v, tan(beta)^2+1)
% syms x y v w 
% [soly, solv] = solve([(x^2 + y^2) * v - x^2 * y^2 * v - 1 == 0,...
%    x^2 - w * y^2 - x^2 * y^2 == 0], [y, v]);
% % We can get:
% soly = +/- x*(1/(x^2 + w))^(1/2);
% solv = (x^2 + w)/(w*x^2 + x^2);

%% q11 (x) and beta (v,(tan(beta)^2 + 1)) as inputs to calculate the output cos_q11 (x) and gamma (w, tan(gamma)^2)
% syms x y v w 
% [soly, solw] = solve([(x^2 + y^2) * v - x^2 * y^2 * v - 1 == 0,...
%    x^2 - w * y^2 - x^2 * y^2 == 0], [y, w]);
% % We can get:
% soly = -/+ ((v*x^2 - 1)/(v*(x - 1)*(x + 1)))^(1/2);
% solw = -(v*x^2 - x^2)/(v*x^2 - 1);

%% beta (v, tan(beta)^2+1) and gamma (w, tan(gamma)^2) as inputs to calculate the output cos_q11 (x) and cos_theta(y) 
% syms x y v w 
% [solx, soly] = solve([(x^2 + y^2) * v - x^2 * y^2 * v - 1 == 0,...
%    x^2 - w * y^2 - x^2 * y^2 == 0], [x, y]);


%%
 function [EulerAngle_q11_theta] = EulerAngles_theta_q11_IK(theta, q11)


       %% q11 and theta  as inputs to calculate the output EulerAngle
       
       u_RotationAxis = [cos(q11), sin(q11), 0];
       r = [u_RotationAxis, theta];
       %-- m=vrrotvec2mat(r):Convert rotation from axis-angle to matrix representation--
       RotationMatrix_from_axis_angle = vrrotvec2mat(r);
       %-- eul = tform2eul(tform): Extract Euler angles from homogeneous transformation--
       eul_alpha_beta_gamma = rotm2eul(RotationMatrix_from_axis_angle,'ZYX');
       
       EulerAngle_q11_theta = [ eul_alpha_beta_gamma, q11, theta ];
 

%             %----------------- plot xyz axes of base point --------------
%             x_axis = [0.5 0 0];
%             y_axis = [0 0.5 0];
%             z_axis = [0 0 0.5];
%             OP= [0 0 0]; 
%             p = [1 1 1];
%             xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
%                 i = 1:2;
%                 plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r','LineWidth',10); hold on
%                 i = 3:4;
%                 plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g','LineWidth',10); hold on
%                 i = 5:6;
%                 plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b','LineWidth',10); hold on
%             %-----------------------------------------------------------
%             %------------------plot xyz axes of Moving Platform----------------
% %             RotationMatrix_from_axis_angle1 = eul2rotm(EulerAngle_q11_theta_1stSolution(1:3),'ZYX');
%             RotationMatrix_from_axis_angle1 = eul2rotm(eul_alpha_beta_gamma,'ZYX');
%             
%             xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(RotationMatrix_from_axis_angle1 * transpose(xyz));
%             i = 1:2;
%             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r','LineWidth',10);
%             i = 3:4;
%             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g','LineWidth',10);   
%             i = 5:6;
%             plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b','LineWidth',10);   
%             hold on;
%             axis equal;
%             xlim([-0.1 1.5])
%             ylim([-0.1 1.5])
%             zlim([-0.1 1.5])
%             %----------------------------------------------

end