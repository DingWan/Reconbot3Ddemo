%
clf
clc
L1 = 230.0692;
L2 = 146.25;
deg = pi/180;
addpath(genpath(pwd)); % Enalbe all folders

PosOri = {0 0 253.3124 0 [] [], pi/2 -pi/2};
q11q12q21q22 = [1.2*pi/3, 1*pi/3, 1.0*pi/4, 1.1*pi/3];
obj3T1R = RCB3T1R(PosOri, q11q12q21q22, L1, L2);
[p_previous, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;

%% Calculate the Jacobian
deltaq = 0.000001;
Num_joint_variables = 4;
delta_q = deltaq * eye(Num_joint_variables);
J_dx_dq_rpy = zeros(6, Num_joint_variables);
for i = 1:Num_joint_variables
    % Previous
    obj3T1R = RCB3T1R(PosOri, q11q12q21q22, L1, L2);
    [p_previous, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;
    % Current
    obj3T1R = RCB3T1R(PosOri, q11q12q21q22 + delta_q(i,:), L1, L2);
    [p_current, ~, ~] = obj3T1R.RCB_3T1R_FK;
    df(:,i) = p_current - p_previous;
    J_dx_dq_rpy(:,i) = [  df(1,i) / deltaq; %This whole thing is a single column
                          df(2,i) / deltaq;
                          df(3,i) / deltaq;
                          df(4,i) / deltaq;
                          df(5,i) / deltaq;
                          df(6,i) / deltaq
                        ];
end

q11 = q1q2(1); q12 = q1q2(2); q13 = q1q2(3); q14 = q1q2(4); q15 = q1q2(5);

q21 = q1q2(6); q22 = q1q2(7); q23 = q1q2(8); q24 = q1q2(9); q25 = q1q2(10);

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

norm(C2 - C1)

%% ===========  Roll/pitch/yaw to homogenous transform =============
yaw = p_previous(4) * 180/pi;
pitch = p_previous(5) * 180/pi;
roll = p_previous(6) * 180/pi;
RotationMatrix = rotz(roll) * roty(pitch) * rotx(yaw);  
C1_Ob = (RotationMatrix * A1')' + p_previous(1:3)

%% All joint screws are defined with respect to instantaneous frame on the moving platform
ABC_op = (ABC - ones(6,1) * p_previous(1:3)) * RotationMatrix;

%%  ============================  Vector ============================ 

% In base frame Ob-XYZ
op_Ob = p_previous(1:3);
%----- Branch Chain A1C1------
opC1_Ob = C1 - op_Ob;
A1B1_Ob = B1 - A1;
B1C1_Ob = C1 - B1;
A1C1_Ob = C1 - A1;
opA1_Ob = A1 - op_Ob;
if C1(1) - A1(1) <= 0
    z_A1D1_Ob = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1_Ob = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
A1D1_Ob = [0, 0, z_A1D1_Ob];
opD1_Ob = opC1_Ob - A1B1_Ob - B1C1_Ob + A1D1_Ob;
%------ Branch Chain A2C2------
opC2_Ob = C2 - op_Ob;
A2B2_Ob = B2 - A2;
B2C2_Ob = C2 - B2;
A2C2_Ob = C2 - A2;
opA2_Ob = A2 - op_Ob;
if C2(1) - A2(1) <= 0
    z_A2D2_Ob = C2(3) + sqrt((C2(1) - A2(1))^2 + (C2(2) - A2(2))^2) * tan(q12 + q13 + q14);
else
    z_A2D2_Ob = C2(3) - sqrt((C2(1) - A2(1))^2 + (C2(2) - A2(2))^2) * tan(q12 + q13 + q14);
end
A2D2_Ob = [0, 0, z_A2D2_Ob];
opD2_Ob = opC2_Ob - A2B2_Ob - B2C2_Ob + A2D2_Ob;

% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 
% In instantaneous frame on the Moving Platform Op-xyz
A1_op = ABC_op(1,:); B1_op = ABC_op(2,:); C1_op = ABC_op(3,:);
A2_op = ABC_op(4,:); B2_op = ABC_op(5,:); C2_op = ABC_op(6,:);
op_op = [0 0 0];
%----- Branch Chain A1C1------
opC1_op = C1_op - op_op;
A1B1_op = B1_op - A1_op;
B1C1_op = C1_op - B1_op;
A1C1_op = C1_op - A1_op;
opA1_op = A1_op - op_op;
A1D1_op = (A1D1_Ob - p_previous(1:3)) * RotationMatrix;
opD1_op = opC1_op - A1B1_op - B1C1_op + A1D1_op;
%------ Branch Chain A2C2------
opC2_op = C2_op - op_op;
A2B2_op = B2_op - A2_op;
B2C2_op = C2_op - B2_op;
A2C2_op = C2_op - A2_op;
opA2_op = A2_op - op_op;
A2D2_op = (A2D2_Ob - p_previous(1:3)) * RotationMatrix;
opD2_op = opC2_op - A2B2_op - B2C2_op + A2D2_op;


%% ========================= screw unit =========================

% In base frame Ob-XYZ
%----- Branch Chain A1C1------
s11_Ob = [0, 0, 1];
s12_Ob = [cos(q11), sin(q11), 0];
s13_Ob = [cos(q11), sin(q11), 0];
s14_Ob = [cos(q11), sin(q11), 0];
s15_Ob = [sin(q11)*cos(q12 + q13 + q14), cos(q11)*sin(q12 + q13 + q14), cos(q12 + q13 + q14)];
sr11_Ob = cross((opC1_Ob - A1B1_Ob - B1C1_Ob), s11_Ob);
sr12_Ob = cross((opC1_Ob - A1B1_Ob - B1C1_Ob), s12_Ob);
sr13_Ob = cross((opC1_Ob - A1B1_Ob), s13_Ob);
sr14_Ob = cross(opC1_Ob, s14_Ob);
sr15_Ob = cross(opC1_Ob, s15_Ob);
%------ Branch Chain A2C2------
s21_Ob = [0, 0, 1];
s22_Ob = [cos(q21), sin(q21), 0];
s23_Ob = [cos(q21), sin(q21), 0];
s24_Ob = [cos(q21), sin(q21), 0];
s25_Ob = [sin(q11)*cos(q12 + q13 + q14), cos(q11)*sin(q12 + q13 + q14), cos(q12 + q13 + q14)];
sr21_Ob = cross((opC2_Ob - A2B2_Ob - B2C2_Ob), s21_Ob);
sr22_Ob = cross((opC2_Ob - A2B2_Ob - B2C2_Ob), s22_Ob);
sr23_Ob = cross((opC2_Ob - A2B2_Ob), s23_Ob);
sr24_Ob = cross(opC2_Ob, s24_Ob);
sr25_Ob = cross(opC2_Ob, s25_Ob);

% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 
% In instantaneous frame on the Moving Platform Op-xyz
%----- Branch Chain A1C1------
s11_op = (s11_Ob - p_previous(1:3)) * RotationMatrix;
s12_op = (s12_Ob - p_previous(1:3)) * RotationMatrix;
s13_op = (s13_Ob - p_previous(1:3)) * RotationMatrix;
s14_op = (s14_Ob - p_previous(1:3)) * RotationMatrix;
s15_op = (s15_Ob - p_previous(1:3)) * RotationMatrix;
sr11_op = (sr11_Ob - p_previous(1:3)) * RotationMatrix;
sr12_op = (sr13_Ob - p_previous(1:3)) * RotationMatrix;
sr13_op = (sr13_Ob - p_previous(1:3)) * RotationMatrix;
sr14_op = (sr14_Ob - p_previous(1:3)) * RotationMatrix;
sr15_op = (sr15_Ob - p_previous(1:3)) * RotationMatrix;
%------ Branch Chain A2C2------
s21_op = (s21_Ob - p_previous(1:3)) * RotationMatrix;
s22_op = (s22_Ob - p_previous(1:3)) * RotationMatrix;
s23_op = (s23_Ob - p_previous(1:3)) * RotationMatrix;
s24_op = (s24_Ob - p_previous(1:3)) * RotationMatrix;
s25_op = (s25_Ob - p_previous(1:3)) * RotationMatrix;
sr21_op = (sr21_Ob - p_previous(1:3)) * RotationMatrix;
sr22_op = (sr22_Ob - p_previous(1:3)) * RotationMatrix;
sr23_op = (sr23_Ob - p_previous(1:3)) * RotationMatrix;
sr24_op = (sr24_Ob - p_previous(1:3)) * RotationMatrix;
sr25_op = (sr25_Ob - p_previous(1:3)) * RotationMatrix;


%% ========================= Reciprocal Screws =========================

% In base frame Ob-XYZ
% Jc Common reciprocal screw 
sr11c_Ob = [-sin(q11), cos(q11), 0];
sr12c_Ob = [cos(q11), sin(q11), 0];
sr21c_Ob = [-sin(q21), cos(q21), 0];
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
            cross(opC1_Ob,B1C1_Ob), B1C1_Ob;
            cross(opA1_Ob,A1B1_Ob), A1B1_Ob;
            cross(opC2_Ob,s22_Ob),  s22_Ob; 
            cross(opC2_Ob,B2C2_Ob), B2C2_Ob;
            cross(opC2_Ob,A2C2_Ob), A2C2_Ob;
       ];
Jx2_Ob = [  cross(s14_Ob,s15_Ob),   0 0 0;
            cross(opC1_Ob,B1C1_Ob), B1C1_Ob;
            cross(opA1_Ob,A1B1_Ob), A1B1_Ob;
            cross(s24_Ob,s25_Ob),   0 0 0;
            cross(opC2_Ob,B2C2_Ob), B2C2_Ob;
            cross(opC2_Ob,A2C2_Ob), A2C2_Ob;
       ];
% Jqk
% Case I: Jq1
Jq1_1_Ob = s12_Ob * sr11_Ob' + s11_Ob * cross(opC1_Ob,s12_Ob)';
Jq1_2_Ob = B1C1_Ob * sr12_Ob' + s12_Ob * cross(opC1_Ob,B1C1_Ob)';
Jq1_3_Ob = A1B1_Ob * sr14_Ob' + s14_Ob * cross(opA1_Ob,A1B1_Ob)';
Jq1_4_Ob = s22_Ob * sr21_Ob' + s21_Ob * cross(opC2_Ob,s22_Ob)';
Jq1_5_Ob = B2C2_Ob * sr22_Ob' + s22_Ob * cross(opC2_Ob,B2C2_Ob)';
Jq1_6_Ob = A2C2_Ob * sr23_Ob' + s23_Ob * cross(opC2_Ob,A2C2_Ob)';
% Case II: Jq2
Jq2_1_Ob = [0 0 0] * sr11_Ob' + s11_Ob * cross(s14_Ob,s15_Ob)';
Jq2_2_Ob = B1C1_Ob * sr12_Ob' + s12_Ob * cross(opC1_Ob,B1C1_Ob)';
Jq2_3_Ob = A1B1_Ob * sr14_Ob' + s14_Ob * cross(opA1_Ob,A1B1_Ob)';
Jq2_4_Ob = [0 0 0] * sr21_Ob' + s21_Ob * cross(s24_Ob,s25_Ob)';
Jq2_5_Ob = B2C2_Ob * sr22_Ob' + s22_Ob * cross(opC2_Ob,B2C2_Ob)';
Jq2_6_Ob = A2C2_Ob * sr23_Ob' + s23_Ob * cross(opC2_Ob,A2C2_Ob)';

% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 
% In instantaneous frame on the Moving Platform Op-xyz
% Jc Common reciprocal screw 
sr11c_op = (sr11c_Ob - p_previous(1:3)) * RotationMatrix;
sr12c_op = (sr12c_Ob - p_previous(1:3)) * RotationMatrix;
sr21c_op = (sr21c_Ob - p_previous(1:3)) * RotationMatrix;
sr22c_op = (sr22c_Ob - p_previous(1:3)) * RotationMatrix;
Jc_op = [  sr11c_op, 0 0 0;
           cross(opD1_op,s12_op), s12_op;
           cross(opC1_op,sr11c_op), sr11c_op;
           cross(opC1_op,s11_op), s11_op;       
           sr21c_op, 0 0 0;
           cross(opD2_op,s22_op), s22_op;
           cross(opC2_op,sr21c_op), sr21c_op;
           cross(opC2_op,s21_op), s21_op;
      ];
% Jxk Reciprocal Screw as locking actuate joint
Jx1_op = [  cross(opC1_op,s12_op),  s12_op;
            cross(opC1_op,B1C1_op), B1C1_op;
            cross(opA1_op,A1B1_op), A1B1_op;
            cross(opC2_op,s22_op),  s22_op; 
            cross(opC2_op,B2C2_op), B2C2_op;
            cross(opC2_op,A2C2_op), A2C2_op;
       ];
Jx2_op = [  cross(s14_op,s15_op),   0 0 0;
            cross(opC1_op,B1C1_op), B1C1_op;
            cross(opA1_op,A1B1_op), A1B1_op;
            cross(s24_op,s25_op),   0 0 0; 
            cross(opC2_op,B2C2_op), B2C2_op;
            cross(opC2_op,A2C2_op), A2C2_op;
       ];
% Jqk
% Case I: Jq1
Jq1_1_op = s12_op * sr11_op' + s11_op * cross(opC1_op,s12_op)';
Jq1_2_op = B1C1_op * sr12_op' + s12_op * cross(opC1_op,B1C1_op)';
Jq1_3_op = A1B1_op * sr14_op' + s14_op * cross(opA1_op,A1B1_op)';
Jq1_4_op = s22_op * sr21_op' + s21_op * cross(opC2_op,s22_op)';
Jq1_5_op = B2C2_op * sr22_op' + s22_op * cross(opC2_op,B2C2_op)';
Jq1_6_op = A2C2_op * sr23_op' + s23_op * cross(opC2_op,A2C2_op)';
% Case II: Jq2
Jq2_1_op = [0 0 0] * sr11_op' + s11_op * cross(s14_op,s15_op)';
Jq2_2_op = B1C1_op * sr12_op' + s12_op * cross(opC1_op,B1C1_op)';
Jq2_3_op = A1B1_op * sr14_op' + s14_op * cross(opA1_op,A1B1_op)';
Jq2_4_op = [0 0 0] * sr21_op' + s21_op * cross(s24_op,s25_op)';
Jq2_5_op = B2C2_op * sr22_op' + s22_op * cross(opC2_op,B2C2_op)';
Jq2_6_op = A2C2_op * sr23_op' + s23_op * cross(opC2_op,A2C2_op)';


%% ========================= Jacobian Matrix Calculation =========================

% In base frame Ob-XYZ
% 3T1R Mode
Jxc1_Ob = [   Jx1_Ob(1,:);
              Jx1_Ob(2,:);
              Jx1_Ob(4,:);
              Jx1_Ob(5,:);
              Jc_Ob(1,:);
              Jc_Ob(5,:);
        ];
Jq1_Ob = [  Jq1_1_Ob   0          0           0
            0          Jq1_2_Ob   0           0 
            0          0          Jq1_4_Ob    0
            0          0          0           Jq1_5_Ob
            0          0          0           0
            0          0          0           0
       ];
   
% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 
% In instantaneous frame on the Moving Platform Op-xyz
% 3T1R Mode
Jxc1_op = [   Jx1_op(1,:);
              Jx1_op(2,:);
              Jx1_op(4,:);
              Jx1_op(5,:);
              Jc_op(1,:);
              Jc_op(5,:);
           ];
Jq1_op = [  Jq1_1_op   0          0           0
            0          Jq1_2_op   0           0 
            0          0          Jq1_4_op    0
            0          0          0           Jq1_5_op
            0          0          0           0
            0          0          0           0
          ]; 

% +++++++++++++++++++++++++++ Elegant Line ++++++++++++++++++++++++++++ 
J_dx_dq_rpy
J_Ob = inv(Jxc1_Ob) * Jq1_Ob
J_Op = inv(Jxc1_op) * Jq1_op
