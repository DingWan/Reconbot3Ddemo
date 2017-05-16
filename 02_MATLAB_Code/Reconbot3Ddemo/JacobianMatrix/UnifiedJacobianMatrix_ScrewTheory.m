

%% =========== Jacobian Matrix by using screw theory ===========
%q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
tic 
%% ----------------Get the output values of Moving Platform---------------
A1 = [0, -L1/2, 0];
B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
C1 = [L2 * (cos(q12) + cos(q12 + q13)) * sin(q11), -L1/2 - L2 * (cos(q12)...
    + cos(q12 + q13)) * cos(q11), L2 * (sin(q12) + sin(q12 + q13))];

A2 = [0, L1/2, 0];
B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
C2 = [- L2 * (cos(q22) + cos(q22 + q23)) * sin(q21), L1/2 + L2 * (cos(q22)...
    + cos(q22 + q23)) * cos(q21), L2 * (sin(q22) + sin(q22 + q23))];

%% ===========  Euler Angle to homogenous transform =============
alpha = p_previous(4) * 180/pi;
beta = p_previous(5) * 180/pi;
gamma = p_previous(6) * 180/pi;
RotationMatrix = eul2rotm(p_previous(4:6)); 

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
Jx1_Ob_3T1R = [    Jx1_Ob(1,:);
                   Jx1_Ob(2,:);
                   Jx1_Ob(4,:);
                   Jx1_Ob(5,:);
                ];
Jq1_Ob_3T1R = [  Jq1_1_Ob   0          0           0
                 0          Jq1_2_Ob   0           0 
                 0          0          Jq1_4_Ob    0
                 0          0          0           Jq1_5_Ob
              ];
Ja_Ob_3T1R = inv(Jq1_Ob_3T1R) * Jx1_Ob_3T1R;
J_Ob_3T1R = [   Ja_Ob_3T1R;
                Jc_Ob(1,:);
                Jc_Ob(5,:);
             ];
% det_Jq1_Ob_3T1R = det(Jq1_Ob_3T1R/norm(Jq1_Ob_3T1R)) % normized
% det_J_Ob_3T1R = det(J_Ob_3T1R)
% det_Jc_Ob_3T1R = det(J_Ob_3T1R(5:6,1:2)/norm(J_Ob_3T1R(5:6,1:2))) % normized