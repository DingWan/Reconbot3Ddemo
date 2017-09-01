function [ a0,a1,a2,a3,a4,a5,T ] = PTP_Polynom5(p, v, a, t)
% Summary of this function goes here
%   Detailed explanation goes here
q0 = p(:,1);
q1 = p(:,2);
t0 = t(1);
t1 = t(2);
v0 = v(:,1);
v1 = v(:,2);
acc0 = a(:,1);
acc1 = a(:,2);
T = t1 - t0;
h = q1 - q0;

a0 = zeros(size(p,1),size(p,2)-1);
a1 = zeros(size(p,1),size(p,2)-1);
a2 = zeros(size(p,1),size(p,2)-1);
a3 = zeros(size(p,1),size(p,2)-1);
a4 = zeros(size(p,1),size(p,2)-1);
a5 = zeros(size(p,1),size(p,2)-1);

a0(:,1) = p(:,1);
a1(:,1) = v0;
a2(:,1) = 1/2 * acc0;
a3(:,1) = (1/(2*T^3)) * (20 * h - (8 * v1+ 12 * v0 ) * T - (3 * acc0 - acc1 ) * T^2 );
a4(:,1) = (1/(2*T^4)) * (- 30 * h + (14 * v1+ 16 * v0 ) * T + (3 * acc0 - 2 * acc1 ) * T^2 );
a5(:,1) = (1/(2*T^5)) * (12 * h - 6*(v1 + v0) * T - ( acc1 - acc0 ) * T^2 ); 

end
