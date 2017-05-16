clf
clc
clear 

q11 = 0;
q12 = q11;

s12 = [ cos(q11),sin(q11) 0];
s22 = [ -cos(q12), -sin(q12), 0];

theta_start = -pi/3;

e1 = cos(theta_start/2);
e2 = s12(1)*sin(theta_start/2);
e3 = s12(2)*sin(theta_start/2);
e4 = s12(3)*sin(theta_start/2);
quaternion_start =  quaternion( [ e1  e2  e3  e4 ] );

theta_end = pi/3;
e1 = cos(theta_end/2);
e2 = s12(1)*sin(theta_end/2);
e3 = s12(2)*sin(theta_end/2);
e4 = s12(3)*sin(theta_end/2);
quaternion_end = quaternion( [ e1  e2  e3  e4 ] );


qn_start = quaternion_start.normalize;
qn_end = quaternion_end.normalize;

[angle, axis] = qn_start.AngleAxis
[angle, axis] = qn_end.AngleAxis

i = 0;
for t = 0: 0.01 : 1;
    i = i + 1;
    qs(i) = slerp( qn_start, qn_end, t );
    PlotRotation(qs(i),0.01);
end

% PlotRotation(qn_start); hold on
% PlotRotation(qn_end); hold on

 %% 5-Grade Polynomial Intepotation
PosOri_previous = [100  100  150  -pi/6];
PosOri_current = [100  -100  150   pi/6];
 
for i = 1:length(PosOri_previous)
 PO = [PosOri_previous(i),PosOri_current(i)];
 v = [0, 0];
 a = [0, 0];
 
 [ a0,a1,a2,a3,a4,a5,T ] =  PTP_Polynom5(PO, v, a, Time);
 
 t = linspace(0,T,n);
 px = [a5,a4,a3,a2,a1,a0];
 %px = [a0,a1,a2,a3,a4,a5];
 pxd = polyder(px);
 pxdd = polyder(pxd);
 Pos = polyval(px,t);
 Vel = polyval(pxd,t);
 Acc = polyval(pxdd,t);
 time_Intep = t + Time(1)*ones(1,n);
 
 Pos_Intep(i,:) = Pos;
 Vel_Intep(i,:) = Vel;
 Acc_Intep(i,:) = Acc;
end
 
 