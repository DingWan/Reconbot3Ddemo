clf
l1 = 230.0692;
l2 = 146.25;
deg = pi/180;
PosOri = {0 0 253.3124 0 [] [], pi/2 -pi/2};
%% --3T1R--
% q11q12q21q22 = [pi/3, pi/3, -pi/6, pi/3];
% obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
% [p, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;

%% --2T2R--
q11q12q14q23 = [pi/3, 0.5*pi/4,1*pi/6, 1*pi/3];
obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
[p, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
grid on
axis equal