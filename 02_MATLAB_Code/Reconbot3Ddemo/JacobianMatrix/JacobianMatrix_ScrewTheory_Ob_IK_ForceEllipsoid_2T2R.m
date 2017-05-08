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

po = {50 120 120 [] [] -30*pi/180};
q11q12q14q23 = [1*pi/4, 0*pi/6, -1*pi/6, 1*pi/3];
% Previous
obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
[p_previous, ~, ~, q1q2_all, ~] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
SelectedRow = find(abs(q1q2_all(:,1)) == min(abs(q1q2_all(:,1))));
q1q2 = q1q2_all(SelectedRow,:);
q1q2_previous = q1q2;

%% initial Position
q11 = q1q2(1); q12 = q1q2(2); q13 = q1q2(3); q14 = q1q2(4); q15 = q1q2(5);
q21 = q1q2(6); q22 = q1q2(7); q23 = q1q2(8); q24 = q1q2(9); q25 = q1q2(10);
 
q0q1q2 = [0, q1q2];
% InitHome;  
ReconbotANI(q0q1q2);

%% =========== Jacobian Matrix by using screw theory ===========
%q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];

% L1 = 0.2300692;
% L2 = 0.14625;
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

% ------- 2T2R ----------
Jx2_Ob_2T2Rsixbar = [   Jx2_Ob(1,:)
                        Jx2_Ob(2,:)
                        Jx2_Ob(3,:)
                        Jx2_Ob(6,:)
                     ];
Jq2_Ob_2T2Rsixbar = [  Jq2_1_Ob   0          0           0
                       0          Jq2_2_Ob   0           0 
                       0          0          Jq2_3_Ob    0
                       0          0          0           Jq2_6_Ob
                    ];
Ja_Ob_2T2Rsixbar = inv(Jq2_Ob_2T2Rsixbar) * Jx2_Ob_2T2Rsixbar;  
J_Ob_2T2Rsixbar = [   Ja_Ob_2T2Rsixbar;
                      Jc_Ob(2,:);
                      Jc_Ob(6,:);
                  ]; 
det(J_Ob_2T2Rsixbar);    
JT_inv = inv(J_Ob_2T2Rsixbar');
Tor_q1q2 = JT_inv /1000 * [ 1 1 1 100 100 100 ]';   

    
% ------ 2T2R redundent ------
Jx2_Ob_2T2Rsixbar_r = [   Jx2_Ob(1,:)
                          Jx2_Ob(2,:)
                          Jx2_Ob(3,:)
                          Jx2_Ob(4,:)
                          Jx2_Ob(5,:)
                          Jx2_Ob(6,:)
                       ];
Jq2_Ob_2T2Rsixbar_r = [  Jq2_1_Ob   0          0           0           0           0
                         0          Jq2_2_Ob   0           0           0           0 
                         0          0          Jq2_3_Ob    0           0           0
                         0          0          0           Jq2_4_Ob    0           0
                         0          0          0           0           Jq2_5_Ob    0
                         0          0          0           0           0           Jq2_6_Ob
                      ]; 
%det(Jx2_Ob_2T2Rsixbar_r)
Ja_Ob_2T2Rsixbar_r = inv(Jq2_Ob_2T2Rsixbar_r) * Jx2_Ob_2T2Rsixbar_r;  
J_Ob_2T2Rsixbar_r = [   Ja_Ob_2T2Rsixbar_r;
                      Jc_Ob(2,:);
                      Jc_Ob(6,:);
                  ]   
    
JT_plus = J_Ob_2T2Rsixbar_r * inv(J_Ob_2T2Rsixbar_r' * J_Ob_2T2Rsixbar_r);
Tor_q1q2 = JT_plus /1000 * [ 1 1 1 100 100 100 ]'
    
% ---------------------- 
    %Torque_UpBoundary
    TorUpBo = 6.0; %N.m    
    
    i_theta = 0;
    j_phi = 0;
    Fn_Fx_Fy_Fz_theta_phi = [];
    Tor_q1q2_Max = [];
    for theta = 0 : 5*pi/180 : pi
        for phi = 0 : 5*pi/180 : 2*pi
            j_phi = j_phi + 1;
            Fn = 0;
            Tor_q11 = 0;
            Tor_q12 = 0;
            Tor_q14 = 0;
            Tor_q21 = 0;
            Tor_q22 = 0;
            Tor_q23 = 0;
            while( abs(Tor_q11) < TorUpBo && abs(Tor_q12) < TorUpBo && abs(Tor_q14) < TorUpBo &&...
                    abs(Tor_q21) < TorUpBo && abs(Tor_q22) < TorUpBo && abs(Tor_q23) < TorUpBo )
                
                Fn = Fn + 1;
                
                Fx = Fn * sin(theta) * cos(phi);
                Fy = Fn * sin(theta) * sin(phi);
                Fz = Fn * cos(theta);
                
                Tor_q1q2 = JT_plus /1000 * [ 1 1 1 Fx Fy Fz ]';
                %Tor_q1q2 = JT_inv /1000 * [ 1 1 1 Fx Fy Fz ]';
                
                Tor_q11 = Tor_q1q2(1);
                Tor_q12 = Tor_q1q2(2);
                Tor_q14 = Tor_q1q2(3);
                %Tor_q21 = Tor_q1q2(4);
                %Tor_q22 = Tor_q1q2(5);
                Tor_q23 = Tor_q1q2(6);
                
            end
            Fn_Fx_Fy_Fz_theta_phi(j_phi,:) = [ Fn Fx Fy Fz theta phi ];
            Tor_q1q2_Max(j_phi,:) = Tor_q1q2;
        end
    end
    Fx_Fy_Fz = Fn_Fx_Fy_Fz_theta_phi(:,2:4);
    figure(2)
    plot3(Fx_Fy_Fz(:,1), Fx_Fy_Fz(:,2), Fx_Fy_Fz(:,3), 'b.')
    xlabel('x');
    ylabel('y');
    zlabel('z');
    grid on  
    axis equal

