clear
clc

% Initial Condition
l1 = 0.2301390;
l2 = 0.1477;
deg = pi/180;
PosOri = {0 0 0.208879343162506 0 [] [], pi/2 -pi/2};
n = 50;
Time = [0,5];

addpath(genpath(pwd));

RCB_CoSim;

%InitHome;

%% Input values [q0, q1, q2, q3, q4]
%3T1R; q11q12q21q23 = []
Mode = 2;
q1q2_Start = [  0*pi/180,   0*pi/180,   45*pi/180,    0*pi/180,    45*pi/180];
  q1q2_End = [  0*pi/180,   0*pi/180,   60*pi/180,    0*pi/180,    60*pi/180];

%2T2R; q11q12q14q23 = []
% q1q2_Start = [  0*pi/180,   0*pi/180,   45*pi/180,  -45*pi/180,   90*pi/180];
%   q1q2_End = [ 30*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   90*pi/180];
  
%2T2Rthreebar;  q11q12q14q23 = []
% q1q2_Start = [  0*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   90*pi/180];
%   q1q2_End = [ 30*pi/180,  90*pi/180,   60*pi/180,  -30*pi/180,   120*pi/180]; 

%2T2Rthreebar;  q11q12q14q23 = []
% q1q2_Start = [  0*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   45*pi/180];
%   q1q2_End = [ 60*pi/180,  90*pi/180,   60*pi/180,  -90*pi/180,    0*pi/180]; 

%RCB2RserialA1C1;  q11q12q22q13 = []
% q1q2_Start = [  0*pi/180,  180*pi/180,    0*pi/180,   45*pi/180,   180*pi/180];
%   q1q2_End = [  0*pi/180,  150*pi/180,   30*pi/180,  120*pi/180,   180*pi/180];   
  
% %RCB2RserialA2C2; q12q21q22q23 = []
% q1q2_Start = [  0*pi/180,  45*pi/180,    0*pi/180,   0*pi/180,   180*pi/180];
%   q1q2_End = [  0*pi/180,  90*pi/180,   30*pi/180,  30*pi/180,   180*pi/180]; 

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

Step = 1:n;

for i = 1:n
    switch Mode
        case 1
            %% --3T2R--
            q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 2
            %% --3T1R--
            q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 3
            %% --RCB_3T1R_SingularityA1C1--
            q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj3T1RSingularityA1C1 = RCB3T1RSingularityA1C1(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1.RCB_3T1R_SingularityA1C1_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 4
            %% --RCB_3T1R_SingularityA2C2--
            q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj3T1RSingularityA2C2 = RCB3T1RSingularityA2C2(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA2C2.RCB_3T1R_SingularityA2C2_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 5
            %% --RCB_3T1R_SingularityA1C1A2C2--
            q11q12q21q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj3T1RSingularityA1C1A2C2 = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1A2C2.RCB_3T1R_SingularityA1C1A2C2_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 6
            %% --2T2R--
            q11q12q14q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 7
            %% --RCB1T2RRotAroundPoint--
            q11q12q14q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj1T2RRotAroundPoint = RCB1T2RRotAroundPoint(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj1T2RRotAroundPoint.RCB_2T2Rsixbar_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 8
            %% --2T2RfiveBar--
            q11q12q14q22 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj2T2Rfivebar = RCB2T2Rfivebar(PosOri, q11q12q14q22 , l1, l2);
            [p, ~, q1q2] = obj2T2Rfivebar.RCB_2T2R_FiveBar_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 9
            %% --2T2RthreeBar--
            q11q12q14q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj2T2Rthreebar = RCB2T2Rthreebar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rthreebar.RCB_2T2R_ThreeBar_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 10
            %% --RCB2RserialA1C1--
            q11q12q22q13 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj2RserialA1C1 = RCB2RserialA1C1(PosOri, q11q12q22q13 , l1, l2);
            [p, ~, q1q2] = obj2RserialA1C1.RCB_2R_serialA1C1_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 11
            %% --RCB2RserialA2C2--
            q12q21q22q23 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            obj2RserialA2C2 = RCB2RserialA2C2(PosOri, q12q21q22q23 , l1, l2);
            [p, ~, q1q2] = obj2RserialA2C2.RCB_2R_serialA2C2_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
        case 12
            %% --RCB2RserialA2C2--
            q11q12q21q22 = [Ang_Intep(2,i), Ang_Intep(3,i), Ang_Intep(4,i), Ang_Intep(5,i)];
            objFixedSerialChain = RCBFixedSerialChain(PosOri, q11q12q21q22 , l1, l2);
            [p, ~, q1q2] = objFixedSerialChain.RCB_FixedSerialChain_FK;
            q0q1q2 = [Ang_Intep(1,i), q1q2];
    end
            %ReconbotANI(q0q1q2);
            q0 = q0q1q2(1);
            q11 = q0q1q2(2);
            q12 = q0q1q2(3)-pi/4;
            q14 = q0q1q2(5)+pi/4;
            q21 = q0q1q2(7);
            q22 = q0q1q2(8)-pi/4;
            q23 = q0q1q2(9)-pi/2;
            Slide = 0;
            LeftArmAngle = pi/6 * sin(i*pi/n);
            RightArmAngle = pi/6 * sin(i*pi/n);
            
            q0q1q2SlideLeftRightArm(i,:) = [q0, q11, q12, q14, q21, q22, q23, LeftArmAngle, RightArmAngle, Slide];% * 180/pi;
            
            %q0q1q2SlideLeftRightArm_time(i,:) = [time_Intep(i), q0, q11, q12, q21, q22, q23, LeftArmAngle, RightArmAngle, Slide];
end
            q0q1q2SlideLeftRightArm_time = [time_Intep', q0q1q2SlideLeftRightArm];
            dlmwrite('Inputs_rad.txt',q0q1q2SlideLeftRightArm_time);