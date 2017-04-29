%
clf
clc
L1 = 230.0692;
L2 = 146.25;
deg = pi/180;
addpath(genpath(pwd)); % Enalbe all folders

PosOri = {0 0 253.3124 0 [] [], pi/2 -pi/2};
q11q12q21q22 = [1*pi/4, pi/3, -0.3*pi/4, pi/6];

%Calculate the Jacobian
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

%% Jacobian Matrix by using screw theory
%q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];

%% -----------------Get the output values of Moving Platform-----------------------
%%--------------------Calculate the position of Ai Bi Ci------------------
A1 = [0, -L1/2, 0];
B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12)...
    + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
%%------------------------------------------------------------------------
%%--------------------Calculate the position of Ai Bi Ci------------------
A2 = [0, L1/2, 0];
B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22)...
    + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];
%%------------------------------------------------------------------------
ABC = [ A1; B1; C1; A2; B2; C2 ];

norm(C2 - C1)

%% Roll/pitch/yaw to homogenous transform
yaw = p_previous(4) * 180/pi;
pitch = p_previous(5) * 180/pi;
roll = p_previous(6) * 180/pi;
RotationMatrix = rotz(roll) * roty(pitch) * rotx(yaw);  
C1_Ob = (RotationMatrix * A1')' + p_previous(1:3)

%% All joint screws are defined with respect to instantaneous frame on the moving platform
ABC_op = (ABC - ones(6,1) * p_previous(1:3)) * RotationMatrix

%% Vector
% Branch Chain A1C1
op = p_previous(1:3);
opC1 = C1 - op;
A1B1 = B1 - A1;
B1C1 = C1 - B1;
A1C1 = C1 - A1;
opA1 = A1 - op(1:3);
if C1(1) - A1(1) <= 0
    z_A1D1 = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1 = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
A1D1 = [0, 0, z_A1D1];
opD1 = opC1 - A1B1 - B1C1 + A1D1;

% Branch Chain A2C2
opC2 = (C1 - C2)/2;
A2B2 = B2 - A2;
B2C2 = C2 - B2;
A2C2 = C2 - A2;
opA2 = A2 - op;
if C2(1) - A2(1) <= 0
    z_A1D1 = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1 = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
A2D2 = [0, 0, z_A1D1];
opD2 = opC2 - A2B2 - B2C2 + A2D2;


%% screw unit
% Branch Chain A1C1
s11 = [0, 0, 1];
s12 = [cos(q11), sin(q11), 0];
s13 = [cos(q11), sin(q11), 0];
s14 = [cos(q11), sin(q11), 0];
s15 = [sin(q11)*cos(q12 + q13 + q14), cos(q11)*sin(q12 + q13 + q14), cos(q12 + q13 + q14)];

sr11 = cross((opC1 - A1B1 - B1C1), s11);
sr12 = cross((opC1 - A1B1 - B1C1), s12);
sr13 = cross((opC1 - A1B1), s13);
sr14 = cross(opC1, s14);
sr15 = cross(opC1, s15);

Screw11 = [s11; sr11];
Screw12 = [s12; sr12];
Screw13 = [s13; sr13];
Screw14 = [s14; sr14];
Screw15 = [s15; sr15];

% Branch Chain A2C2
s21 = [0, 0, 1];
s22 = [cos(q21), sin(q21), 0];
s23 = [cos(q21), sin(q21), 0];
s24 = [cos(q21), sin(q21), 0];
s25 = [sin(q11)*cos(q12 + q13 + q14), cos(q11)*sin(q12 + q13 + q14), cos(q12 + q13 + q14)];

sr21 = cross((opC2 - A2B2 - B2C2), s21);
sr22 = cross((opC2 - A2B2 - B2C2), s22);
sr23 = cross((opC2 - A2B2), s23);
sr24 = cross(opC2, s24);
sr25 = cross(opC2, s25);

Screw21 = [s21; sr21];
Screw22 = [s22; sr22];
Screw23 = [s23; sr23];
Screw24 = [s24; sr24];
Screw25 = [s25; sr25];


%% Reciprocal Screws
% Jc Common reciprocal screw 
sr11c = [-sin(q11), cos(q11), 0];
sr12c = [cos(q11), sin(q11), 0];

sr21c = [-sin(q21), cos(q21), 0];
sr22c = [cos(q21), sin(q21), 0];


Jc = [ sr11c, 0 0 0;
       cross(opD1,s12), s12;
       cross(opC1,sr11c), sr11c;
       cross(opC1,s11), s11;
       
       sr21c, 0 0 0;
       cross(opD2,s22), s22;
       cross(opC2,sr21c), sr21c;
       cross(opC2,s21), s21;
      ];

% Jxk Reciprocal Screw as locking actuate joint
Jx1 = [ cross(opC1,s12),  s12;
        cross(opC1,B1C1), B1C1;
        cross(opA1,A1B1), A1B1;
        cross(opC2,s22),  s22; 
        cross(opC2,B2C2), B2C2;
        cross(opC2,A2C2), A2C2;
       ];

Jx2 = [ cross(s14,s15),   0 0 0;
        cross(opC1,B1C1), B1C1;
        cross(opA1,A1B1), A1B1;
        cross(s24,s25),   0 0 0; 
        cross(opC2,B2C2), B2C2;
        cross(opC2,A2C2), A2C2;
       ];

% Jqk
% Case I: Jq1
Jq1_1 = s12 * sr11' + s11 * cross(opC1,s12)';
Jq1_2 = B1C1 * sr12' + s12 * cross(opC1,B1C1)';
Jq1_3 = A1B1 * sr14' + s14 * cross(opA1,A1B1)';
Jq1_4 = s22 * sr21' + s21 * cross(opC2,s22)';
Jq1_5 = B2C2 * sr22' + s22 * cross(opC2,B2C2)';
Jq1_6 = A2C2 * sr23' + s23 * cross(opC2,A2C2)';

% Case II: Jq2
Jq2_1 = [0 0 0] * sr11' + s11 * cross(s14,s15)';
Jq2_2 = B1C1 * sr12' + s12 * cross(opC1,B1C1)';
Jq2_3 = A1B1 * sr14' + s14 * cross(opA1,A1B1)';
Jq2_4 = [0 0 0] * sr21' + s21 * cross(s24,s25)';
Jq2_5 = B2C2 * sr22' + s22 * cross(opC2,B2C2)';
Jq2_6 = A2C2 * sr23' + s23 * cross(opC2,A2C2)';


%% Jacobian Matrix Calculation
% 3T1R Mode
Jxc1 = [  Jx1(1,:);
          Jx1(2,:);
          Jx1(4,:);
          Jx1(5,:);
          Jc(1,:);
          Jc(5,:);
        ];

Jq1 = [ Jq1_1   0       0       0
        0       Jq1_2   0       0 
        0       0       Jq1_4   0
        0       0       0       Jq1_5
        0       0       0       0
        0       0       0       0
       ];

J_dx_dq_rpy
J = inv(Jxc1) * Jq1

