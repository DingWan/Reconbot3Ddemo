%clf
%% 
l1 = 0.2301390;
l2 = 0.1477;
deg = pi/180;
PosOri = {0 0 0.208879343162506 0 [] [], pi/2 -pi/2};
n = 50;
Time = [0,5];
%InitHome;

%% Input values [q0, q1, q2, q3, q4]
q1q2_Start = [ 0*pi/180,  0*pi/180, 45*pi/180,   0*pi/180,  90*pi/180];
  q1q2_End = [30*pi/180, -30*pi/180, 120*pi/180, -45*pi/180, 60*pi/180];

%% 5-Grade Polynomial Intepotation
for i = 1:5
    PO = [q1q2_Start(i),q1q2_End(i)];
    v = [0, 0];
    a = [0, 0];
    
    [ a0,a1,a2,a3,a4,a5,T ] =  PTP_Polynom5(PO, v, a, Time) ;
    
    t = linspace(0,T,n);
    px = [a5,a4,a3,a2,a1,a0];
    pxd = polyder(px);
    pxdd = polyder(pxd);
    Ang = polyval(px,t);
    AngVel = polyval(pxd,t);
    AngAcc = polyval(pxdd,t);
    time_Intep = t + Time(1)*ones(1,n);
    
    Ang_Intep(i,:) = Ang;
    AngVel_Intep(i,:) = AngVel;
    AngAcc_Intep(i,:) = AngAcc;
end

%% --3T1R--
% for i = 1:n
%     q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
%     obj3T1R = RCB3T1R(PosOri, q11q12q21q23, l1, l2);
%     [p, ABC, q1q2] = obj3T1R.RCB_3T1R_FK;
%     q0q1q2 = [Ang_Intep(1,i), q1q2];
%     ReconbotANI(q0q1q2);
% end

%% --2T2R--
q11q12q14q23 = [pi/3, 0.5*pi/4,1*pi/6, 1*pi/3];
obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
[p, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
grid on
axis equal
