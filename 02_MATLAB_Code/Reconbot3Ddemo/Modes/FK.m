clf
l1 = 230.0692;
l2 = 146.25;
deg = pi/180;
PosOri = {0 0 253.3124 0 [] [], pi/2 -pi/2};
q11q12q21q22 = [pi/3, pi/3, -pi/6, pi/3];
obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
[p, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;
grid on
axis equal
