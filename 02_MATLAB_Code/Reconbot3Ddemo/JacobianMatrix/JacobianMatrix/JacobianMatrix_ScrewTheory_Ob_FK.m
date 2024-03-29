%
%clf
clc
clear

L1 = 230.0692;
L2 = 146.25;
l1 = 230.0692;
l2 = 146.25;

deg = pi/180;
addpath(genpath(pwd)); % Enalbe all folders

PosOri = {0 0 253.3124 0 [] [], pi/2 -pi/2};

% --3T1R--
% q11q12q21q22 = [ -0.3527    1.4831   -1.0780    0.4314  ];
%[1*pi/3, 1*pi/4, 1*pi/4, 1*pi/3];
%[0.8143    0.9044    1.9952   -1.3289    1.3379   -1.5117    0.1379    1.9104   -0.4775   -2.0353]
%[-0.3527    1.4118    1.6916   -1.5325    0.1709   -1.0780    0.0510    1.2940    0.2257   -1.6016]
%[-0.3527    1.4831    1.2780   -1.1903    0.1709   -1.0780    0.4314    0.8199    0.3195   -1.6016]
% obj3T1R = RCB3T1R(PosOri, q11q12q21q22, L1, L2);
% [p_previous, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;

% --2T2Rsixbar--
 q11q12q14q23 = [0.0997    1.6930   -0.2004   1.9463];
 % [-0.7854    1.6943    0.9125   -0.7916   -0.7854   -0.7854    0.2354    1.3574   -0.2664   -0.7854]
 %[ 0.0997    1.6930    0.6018   -0.2004    0.0997    0.0997    0.0598    1.9463   -0.9589     0.0997]
% obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
% [p, ~, ~] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;

%% Calculate the FK Jacobian
deltaq = 0.00001;
Num_joint_variables = 4;
delta_q = deltaq * eye(Num_joint_variables);
J_dq2dx_eul = zeros(6, Num_joint_variables);
% --3T1R--
% % Previous
% obj3T1R = RCB3T1R(PosOri, q11q12q21q22, L1, L2);
% [p_previous, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;
% -- 2T2Rsixbar--
% Previous
obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
[p_previous, ~, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
for i = 1:Num_joint_variables
    % --3T1R--
%     % Current
%     obj3T1R = RCB3T1R(PosOri, q11q12q21q22 + delta_q(i,:), L1, L2);
%     [p_current, ~, ~] = obj3T1R.RCB_3T1R_FK;
    
    % -- 2T2Rsixbar--
    % Current
    obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 + delta_q(i,:), l1, l2);
    [p_current, ~, ~] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;    
    
    df(:,i) = p_current - p_previous;
    J_dq2dx_eul(:,i) = [  df(1,i) / deltaq; %This whole thing is a single column
                          df(2,i) / deltaq;
                          df(3,i) / deltaq;
                          df(4,i) / deltaq;
                          df(5,i) / deltaq;
                          df(6,i) / deltaq
                        ];
end

% obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 + deltaq * ones(1,Num_joint_variables), l1, l2);
% [p_current, ~, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
% (p_current - p_previous)'
% J_dx_dq_eul * (deltaq * ones(1,Num_joint_variables))'


%% initial Position
q11 = q1q2(1); q12 = q1q2(2); q13 = q1q2(3); q14 = q1q2(4); q15 = q1q2(5);
q21 = q1q2(6); q22 = q1q2(7); q23 = q1q2(8); q24 = q1q2(9); q25 = q1q2(10);
 
q0q1q2 = [0, q1q2];
% InitHome;  
ReconbotANI(q0q1q2);

%% =========== Jacobian Matrix by using screw theory ===========
%q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];

%% ----------------Get the output values of Moving Platform---------------
%%--------------------Calculate the position of Ai Bi Ci------------------
A1 = [0, -L1/2, 0];
B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12)...
    + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];

A2 = [0, L1/2, 0];
B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22)...
    + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
%%------------------------------------------------------------------------
ABC = [ A1; B1; C1; A2; B2; C2 ];
%% --------------------Plot the mechanism Ai_Disp Bi_Disp Ci_Disp------------------
% Displacement = [250,250,167.4400];
% A1_Disp = (rotz(90) * A1')'  + Displacement;
% B1_Disp = (rotz(90) * B1')'  + Displacement;
% C1_Disp = (rotz(90) * C1')'  + Displacement;
% A2_Disp = (rotz(90) * A2')'  + Displacement;
% B2_Disp = (rotz(90) * B2')'  + Displacement;
% C2_Disp = (rotz(90) * C2')'  + Displacement;

% PA1B1C1x = [A1_Disp(1), B1_Disp(1), C1_Disp(1)];
% PA1B1C1y = [A1_Disp(2), B1_Disp(2), C1_Disp(2)];
% PA1B1C1z = [A1_Disp(3), B1_Disp(3), C1_Disp(3)];
% plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
% 
% PA2B2C2x = [A2_Disp(1), B2_Disp(1), C2_Disp(1)];
% PA2B2C2y = [A2_Disp(2), B2_Disp(2), C2_Disp(2)];
% PA2B2C2z = [A2_Disp(3), B2_Disp(3), C2_Disp(3)];
% plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
% 
% PC1C2x = [C1_Disp(1), C2_Disp(1)];
% PC1C2y = [C1_Disp(2), C2_Disp(2)];
% PC1C2z = [C1_Disp(3), C2_Disp(3)];
% plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
% 
% PA1A2x = [A1_Disp(1), A2_Disp(1)];
% PA1A2y = [A1_Disp(2), A2_Disp(2)];
% PA1A2z = [A1_Disp(3), A2_Disp(3)];
% plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
%     

%% ===========  Euler Angle to homogenous transform =============

alpha = p_previous(4) * 180/pi;
beta = p_previous(5) * 180/pi;
gamma = p_previous(6) * 180/pi;
RotationMatrix = eul2rotm(p_previous(4:6));  
% C1_Ob = (RotationMatrix * A1')' + p_previous(1:3);
% C2_Ob = (RotationMatrix * A2')' + p_previous(1:3);
% norm(C2_Ob-C1_Ob)

%%  ============================  Vector ============================ 

% In base frame Ob-XYZ
op_Ob = p_previous(1:3);
%----- Branch Chain A1C1------
opA1_Ob = A1 - op_Ob;
opB1_Ob = B1 - op_Ob;
opC1_Ob = C1 - op_Ob;
A1B1_Ob = B1 - A1;
B1C1_Ob = C1 - B1;
A1C1_Ob = C1 - A1;
sr11c_Ob = [-sin(q11), cos(q11), 0];
Angle_A1C1_to_sr11c = acos ( (A1C1_Ob * sr11c_Ob')/(norm(A1C1_Ob)*norm(sr11c_Ob)) ) * 180/pi;
if Angle_A1C1_to_sr11c >= 90
    z_A1D1_Ob = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1_Ob = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
D1A1_Ob = [0, 0, - z_A1D1_Ob];
opD1_Ob = opA1_Ob - D1A1_Ob;
%opD1_Ob = [0 A1(2) z_A1D1_Ob] - op_Ob

% 
% D1A1_Ob_Disp = -D1A1_Ob + Displacement;
% PA1D1C1_Dispx = [A1_Disp(1), A1_Disp(1), C1_Disp(1)];
% PA1D1C1_Dispy = [A1_Disp(2), A1_Disp(2), C1_Disp(2)];
% PA1D1C1_Dispz = [A1_Disp(3), D1A1_Ob_Disp(3), C1_Disp(3)];
% plot3(PA1D1C1_Dispx(:),PA1D1C1_Dispy(:),PA1D1C1_Dispz(:),'b-.');

%------ Branch Chain A2C2------
opA2_Ob = A2 - op_Ob;
opB2_Ob = B2 - op_Ob;
opC2_Ob = C2 - op_Ob;
A2B2_Ob = B2 - A2;
B2C2_Ob = C2 - B2;
A2C2_Ob = C2 - A2;
sr21c_Ob = [sin(q21), -cos(q21), 0];
Angle_A2C2_to_sr21c = acos ( (A2C2_Ob * sr21c_Ob')/(norm(A2C2_Ob)*norm(sr21c_Ob)) ) * 180/pi;
if Angle_A2C2_to_sr21c >= 90
    z_A2D2_Ob = C2(3) - sqrt((C2(1) - A2(1))^2 + (C2(2) - A2(2))^2) * tan(q22 + q23 + q24);
else
    z_A2D2_Ob = C2(3) + sqrt((C2(1) - A2(1))^2 + (C2(2) - A2(2))^2) * tan(q22 + q23 + q24);
end
D2A2_Ob = [0, 0, - z_A2D2_Ob];
opD2_Ob = opA2_Ob - D2A2_Ob;
%opD2_Ob = [0 A2(2) z_A2D2_Ob] - op_Ob
%
% D2A2_Ob_Disp = -D2A2_Ob  + Displacement;
% PA2D2C2_Dispx = [A2_Disp(1), A2_Disp(1), C2_Disp(1)];
% PA2D2C2_Dispy = [A2_Disp(2), A2_Disp(2), C2_Disp(2)];
% PA2D2C2_Dispz = [A2_Disp(3), D2A2_Ob_Disp(3), C2_Disp(3)];
% plot3(PA2D2C2_Dispx(:),PA2D2C2_Dispy(:),PA2D2C2_Dispz(:),'r-.');

%% ========================= screw unit =========================

% In base frame Ob-XYZ
%----- Branch Chain A1C1------
s11_Ob = [0, 0, 1];
s12_Ob = [-cos(q11), -sin(q11), 0];
s13_Ob = [-cos(q11), -sin(q11), 0];
s14_Ob = [-cos(q11), -sin(q11), 0];
s15_Ob = (RotationMatrix * [0 0 1]')';
%[-sin(q11)*cos(q12 + q13 + q14), -cos(q11)*cos(q12 + q13 + q14), sin(q12 + q13 + q14)]
sr11_Ob = cross(opA1_Ob, s11_Ob);
sr12_Ob = cross(opA1_Ob, s12_Ob);
sr13_Ob = cross(opB1_Ob, s13_Ob);
sr14_Ob = cross(opC1_Ob, s14_Ob);
sr15_Ob = cross(opC1_Ob, s15_Ob);
%
% Ob_Ob_origin = Displacement;
% s15_Ob_Disp = (rotz(90) * (s15_Ob*200)')' + Displacement;
% Origin_to_s15_Ob_x = [Ob_Ob_origin(1), s15_Ob_Disp(1)];
% Origin_to_s15_Ob_y = [Ob_Ob_origin(2), s15_Ob_Disp(2)];
% Origin_to_s15_Ob_z = [Ob_Ob_origin(3), s15_Ob_Disp(3)];
% plot3(Origin_to_s15_Ob_x, Origin_to_s15_Ob_y, Origin_to_s15_Ob_z,'G-'); hold on;

%------ Branch Chain A2C2------
s21_Ob = [0, 0, 1];
s22_Ob = [cos(q21), sin(q21), 0];
s23_Ob = [cos(q21), sin(q21), 0];
s24_Ob = [cos(q21), sin(q21), 0];
s25_Ob = (RotationMatrix * [0 0 1]')';
%[sin(q21)*cos(q22 + q23 + q24), cos(q21)*cos(q22 + q23 + q24), sin(q22 + q23 + q24)]
sr21_Ob = cross(opA2_Ob, s21_Ob);
sr22_Ob = cross(opA2_Ob, s22_Ob);
sr23_Ob = cross(opB2_Ob, s23_Ob);
sr24_Ob = cross(opC2_Ob, s24_Ob);
sr25_Ob = cross(opC2_Ob, s25_Ob);

%% ========================= Reciprocal Screws =========================

% In base frame Ob-XYZ
% Jc Common reciprocal screw 
sr11c_Ob = [-sin(q11), cos(q11), 0];
sr12c_Ob = [cos(q11), sin(q11), 0];
sr21c_Ob = [sin(q21), -cos(q21), 0];
sr22c_Ob = [cos(q21), sin(q21), 0];
Jc_Ob = [  sr11c_Ob, 0 0 0;
           cross(opD1_Ob,s12_Ob), s12_Ob;
           cross(opC1_Ob,sr11c_Ob), sr11c_Ob;
           cross(opC1_Ob,s11_Ob), s11_Ob;       
           sr21c_Ob, 0 0 0;
           cross(opD2_Ob,s22_Ob), s22_Ob;
           cross(opC2_Ob,sr21c_Ob), sr21c_Ob;
           cross(opC2_Ob,s21_Ob), s21_Ob;
        ];
% Jxk Reciprocal Screw as locking actuate joint
Jx1_Ob = [  cross(opC1_Ob,s12_Ob),  s12_Ob;
            cross(opC1_Ob,B1C1_Ob/norm(B1C1_Ob)), B1C1_Ob/norm(B1C1_Ob);
            cross(opA1_Ob,A1B1_Ob/norm(A1B1_Ob)), A1B1_Ob/norm(A1B1_Ob);
            cross(opC2_Ob,s22_Ob),  s22_Ob; 
            cross(opC2_Ob,B2C2_Ob/norm(B2C2_Ob)), B2C2_Ob/norm(B2C2_Ob);
            cross(opC2_Ob,A2C2_Ob/norm(A2C2_Ob)), A2C2_Ob/norm(A2C2_Ob);
         ];
Jx2_Ob = [  cross(s14_Ob,s15_Ob),   0 0 0;
            cross(opC1_Ob,B1C1_Ob/norm(B1C1_Ob)), B1C1_Ob/norm(B1C1_Ob);
            cross(opA1_Ob,A1B1_Ob/norm(A1B1_Ob)), A1B1_Ob/norm(A1B1_Ob);
            cross(s24_Ob,s25_Ob),   0 0 0;
            cross(opC2_Ob,B2C2_Ob/norm(B2C2_Ob)), B2C2_Ob/norm(B2C2_Ob);
            cross(opC2_Ob,A2C2_Ob/norm(A2C2_Ob)), A2C2_Ob/norm(A2C2_Ob);
         ];
% Jqk
% Case I: Jq1
Jq1_1_Ob = s12_Ob * sr11_Ob' + s11_Ob * cross(opC1_Ob,s12_Ob)';
Jq1_2_Ob = B1C1_Ob/norm(B1C1_Ob) * sr12_Ob' + s12_Ob * cross(opC1_Ob,B1C1_Ob/norm(B1C1_Ob))';
Jq1_3_Ob = A1B1_Ob/norm(A1B1_Ob) * sr14_Ob' + s14_Ob * cross(opA1_Ob,A1B1_Ob/norm(A1B1_Ob))';
Jq1_4_Ob = s22_Ob * sr21_Ob' + s21_Ob * cross(opC2_Ob,s22_Ob)';
Jq1_5_Ob = B2C2_Ob/norm(B2C2_Ob) * sr22_Ob' + s22_Ob * cross(opC2_Ob,B2C2_Ob/norm(B2C2_Ob))';
Jq1_6_Ob = A2C2_Ob/norm(A2C2_Ob) * sr23_Ob' + s23_Ob * cross(opC2_Ob,A2C2_Ob/norm(A2C2_Ob))';
% Case II: Jq2
Jq2_1_Ob = [0 0 0] * sr11_Ob' + s11_Ob * cross(s14_Ob,s15_Ob)';
Jq2_2_Ob = B1C1_Ob/norm(B1C1_Ob) * sr12_Ob' + s12_Ob * cross(opC1_Ob,B1C1_Ob/norm(B1C1_Ob))';
Jq2_3_Ob = A1B1_Ob/norm(A1B1_Ob) * sr14_Ob' + s14_Ob * cross(opA1_Ob,A1B1_Ob/norm(A1B1_Ob))';
Jq2_4_Ob = [0 0 0] * sr21_Ob' + s21_Ob * cross(s24_Ob,s25_Ob)';
Jq2_5_Ob = B2C2_Ob/norm(B2C2_Ob) * sr22_Ob' + s22_Ob * cross(opC2_Ob,B2C2_Ob/norm(B2C2_Ob))';
Jq2_6_Ob = A2C2_Ob/norm(A2C2_Ob) * sr23_Ob' + s23_Ob * cross(opC2_Ob,A2C2_Ob/norm(A2C2_Ob))';


%% ========================= Jacobian Matrix Calculation =========================

% In base frame Ob-XYZ
% 3T1R Mode
Jxc1_Ob_3T1R = [   Jx1_Ob(1,:);
                   Jx1_Ob(2,:);
                   Jx1_Ob(4,:);
                   Jx1_Ob(5,:);
                   Jc_Ob(1,:);
                   Jc_Ob(5,:);
                ];
Jq1_Ob_3T1R = [  Jq1_1_Ob   0          0           0
                 0          Jq1_2_Ob   0           0 
                 0          0          Jq1_4_Ob    0
                 0          0          0           Jq1_5_Ob
                 0          0          0           0
                 0          0          0           0                 
              ];

         
% 2T2Rsixbar
Jxc2_Ob_2T2Rsixbar = [   Jx2_Ob(1,:);
                         Jx2_Ob(2,:);
                         Jx2_Ob(3,:);
                         Jx2_Ob(6,:);
                         Jc_Ob(2,:);
                         Jc_Ob(6,:);
                     ];
Jq2_Ob_2T2Rsixbar = [  Jq2_1_Ob   0          0           0
                       0          Jq2_2_Ob   0           0 
                       0          0          Jq2_3_Ob    0
                       0          0          0           Jq2_6_Ob
                       0          0          0           0
                       0          0          0           0
                    ];          

% Jx2_Ob(1,1:3) * s12_Ob' + Jx2_Ob(1,4:6) * sr12_Ob' 
% Jx2_Ob(1,1:3) * s13_Ob' + Jx2_Ob(1,4:6) * sr13_Ob' 
% Jx2_Ob(1,1:3) * s14_Ob' + Jx2_Ob(1,4:6) * sr14_Ob' 
% Jx2_Ob(1,1:3) * s15_Ob' + Jx2_Ob(1,4:6) * sr15_Ob'                 
                
% Jx2_Ob(2,1:3) * s11_Ob' + Jx2_Ob(2,4:6) * sr11_Ob'
% Jx2_Ob(2,1:3) * s13_Ob' + Jx2_Ob(2,4:6) * sr13_Ob' 
% Jx2_Ob(2,1:3) * s14_Ob' + Jx2_Ob(2,4:6) * sr14_Ob' 
% Jx2_Ob(2,1:3) * s15_Ob' + Jx2_Ob(2,4:6) * sr15_Ob'

% Jx2_Ob(3,1:3) * s11_Ob' + Jx2_Ob(3,4:6) * sr11_Ob'
% Jx2_Ob(3,1:3) * s12_Ob' + Jx2_Ob(3,4:6) * sr12_Ob' 
% Jx2_Ob(3,1:3) * s13_Ob' + Jx2_Ob(3,4:6) * sr13_Ob' 
% Jx2_Ob(3,1:3) * s15_Ob' + Jx2_Ob(3,4:6) * sr15_Ob' 

% Jx2_Ob(6,1:3) * s21_Ob' + Jx2_Ob(6,4:6) * sr21_Ob'
% Jx2_Ob(6,1:3) * s22_Ob' + Jx2_Ob(6,4:6) * sr22_Ob'  
% Jx2_Ob(6,1:3) * s24_Ob' + Jx2_Ob(6,4:6) * sr24_Ob' 
% Jx2_Ob(6,1:3) * s25_Ob' + Jx2_Ob(6,4:6) * sr25_Ob' 

% Jc_Ob(2,1:3) * s11_Ob' + Jc_Ob(2,4:6) * sr11_Ob'
% Jc_Ob(2,1:3) * s12_Ob' + Jc_Ob(2,4:6) * sr12_Ob' 
% Jc_Ob(2,1:3) * s13_Ob' + Jc_Ob(2,4:6) * sr13_Ob' 
% Jc_Ob(2,1:3) * s14_Ob' + Jc_Ob(2,4:6) * sr14_Ob' 
% Jc_Ob(2,1:3) * s15_Ob' + Jc_Ob(2,4:6) * sr15_Ob' 
% 
% Jc_Ob(6,1:3) * s21_Ob' + Jc_Ob(6,4:6) * sr21_Ob' 
% Jc_Ob(6,1:3) * s22_Ob' + Jc_Ob(6,4:6) * sr22_Ob' 
% Jc_Ob(6,1:3) * s23_Ob' + Jc_Ob(6,4:6) * sr23_Ob' 
% Jc_Ob(6,1:3) * s24_Ob' + Jc_Ob(6,4:6) * sr24_Ob' 
% Jc_Ob(6,1:3) * s25_Ob' + Jc_Ob(6,4:6) * sr25_Ob'
      
% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 

 [U, S, ~] = svd(J_dq2dx_eul')
%
alpha = p_previous(4);
beta = p_previous(5);
gamma = p_previous(6);

T_A = [0 -sin(alpha) cos(alpha)*cos(beta) ;
       0  cos(alpha) sin(alpha)*cos(beta) ;
       1  0          -sin(beta)
       ];
J_dx2dq_eul_Geom = T_A * J_dq2dx_eul(4:6,:);
J_Ob_2T2Rsixbar = inv(Jxc2_Ob_2T2Rsixbar) * Jq2_Ob_2T2Rsixbar



% 3T1R Mode
% J_Ob_3T1R_dq2dx = inv(Jxc1_Ob_3T1R) * Jq1_Ob_3T1R


% 2T2Rsixbar Mode
% Jx2_Ob_2T2Rsixbar = [   Jx2_Ob(1,:)
%                         Jx2_Ob(2,:)
%                         Jx2_Ob(3,:)
%                         Jx2_Ob(6,:)
%                      ];
% Jq2_Ob_2T2Rsixbar = [  Jq2_1_Ob   0          0           0
%                        0          Jq2_2_Ob   0           0 
%                        0          0          Jq2_3_Ob    0
%                        0          0          0           Jq2_6_Ob
%                     ];   
% 
% Ja_Ob_2T2Rsixbar = inv(Jq2_Ob_2T2Rsixbar) * Jx2_Ob_2T2Rsixbar   
% J_Ob_2T2Rsixbar = inv(Jxc2_Ob_2T2Rsixbar) * Jq2_Ob_2T2Rsixbar


% Jc_Ob(2,1:3) * s11_Ob' + Jc_Ob(2,4:6) * sr11_Ob' 
% Jc_Ob(2,1:3) * s12_Ob' + Jc_Ob(2,4:6) * sr12_Ob' 
% Jc_Ob(2,1:3) * s13_Ob' + Jc_Ob(2,4:6) * sr13_Ob' 
% Jc_Ob(2,1:3) * s14_Ob' + Jc_Ob(2,4:6) * sr14_Ob' 
% Jc_Ob(2,1:3) * s15_Ob' + Jc_Ob(2,4:6) * sr15_Ob' 
% cross(opD1_Ob,s12_Ob) * s15_Ob' + s12_Ob * sr15_Ob'

% Jc_Ob(6,1:3) * s12_Ob' + Jc_Ob(6,4:6) * sr12_Ob' 
% Jc_Ob(6,1:3) * s13_Ob' + Jc_Ob(6,4:6) * sr13_Ob' 
% Jc_Ob(6,1:3) * s14_Ob' + Jc_Ob(6,4:6) * sr14_Ob' 
% Jc_Ob(6,1:3) * s15_Ob' + Jc_Ob(6,4:6) * sr15_Ob'
