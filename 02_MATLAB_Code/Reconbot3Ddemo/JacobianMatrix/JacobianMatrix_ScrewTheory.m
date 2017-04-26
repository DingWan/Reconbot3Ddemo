% Jacobian Matrix by using screw theory
q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];

%% -----------------Get the output values of Moving Platform-----------------------
%%--------------------Calculate the position of Ai Bi Ci------------------
A1(:) = [0, -L1/2, 0];
B1(:) = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
C1(:) = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12)...
    + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];
%%------------------------------------------------------------------------
%%--------------------Calculate the position of Ai Bi Ci------------------
A2(:) = [0, L1/2, 0];
B2(:) = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
C2(:) = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22)...
    + cos(q22 + q23)) * cos(q21), L2 * (sin(q22)) + sin(q22 + q23))];
%%------------------------------------------------------------------------
                    
%% Vector
% Branch Chain A1C1
opC1 = (C2 - C1)/2;
A1B1 = B1 - A1;
B1C1 = C1 - B1;
if C1(1) - A1(1) <= 0
    z_A1D1 = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1 = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
A1D1 = [0, 0, z_A1D1];

% Branch Chain A2C2
opC2 = (C1 - C2)/2;
A2B2 = B2 - A2;
B2C2 = C2 - B2;
if C2(1) - A2(1) <= 0
    z_A1D1 = C1(3) + sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
else
    z_A1D1 = C1(3) - sqrt((C1(1) - A1(1))^2 + (C1(2) - A1(2))^2) * tan(q12 + q13 + q14);
end
A2D2 = [0, 0, z_A1D1];


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
sr12c = [-cos(q11), sin(q11), 0];

sr21c = [-sin(q21), cos(q21), 0];
sr22c = [-cos(q21), sin(q21), 0];

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
Jq1_1 = s12 * sr11 + s11 * cross(opC1,s12);
Jq1_2 = B1C1 * sr12 + s12 * cross(opC1,B1C1);
Jq1_3 = A1B1 * sr14 + s14 * cross(opA1,A1B1);

Jq1_4 = s22 * sr21 + s21 * cross(opC2,s22);
Jq1_5 = B2C2 * sr22 + s22 * cross(opC2,B2C2);
Jq1_6 = A2B2 * sr23 + s23 * cross(opC2,A2C2);

% Case II: Jq2
Jq2_1 = [0 0 0] * sr11 + s11 * cross(s14,s15);
Jq2_2 = B1C1 * sr12 + s12 * cross(opC1,B1C1);
Jq2_3 = A1B1 * sr14 + s14 * cross(opA1,A1B1);

Jq2_4 = [0 0 0] * sr21 + s21 * cross(s24,s25);
Jq2_5 = B2C2 * sr22 + s22 * cross(opC2,B2C2);
Jq2_6 = A2B2 * sr23 + s23 * cross(opC2,A2C2);

%% Jacobian Matrix Calculation
Jxc1 = 



