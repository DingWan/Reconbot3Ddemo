l1 = 2.20;
l2 = 1.4725;
% po = { 1    1    1  pi/6  []    []};
% % q0q1q2 = [0, 0, 60, 60, 30, 0, 0, 60, 60, 30, 0];
% q11q12q21q22 = [0, pi/4, 0, pi/3];
% obj3T1R = RCB3T1R(po,q11q12q21q22,l1,l2);
% % [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;
% [p, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;

po = { 1    1    1    []    []    pi/6};
% q0q1q2 = [0, 0, 60, 60, 30, 0, 0, 60, 60, 30, 0];
q11q12q14q23 = [pi/4, pi/4, pi/4, pi/3];
obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
% [p, EulerAngle_q11_theta, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
[p, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;