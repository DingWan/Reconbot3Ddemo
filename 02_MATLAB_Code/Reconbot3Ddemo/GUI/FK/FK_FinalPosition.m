

global Mode;
global q0;
global q11;
global q12;
global q14;
global q21;
global q22;
global q23;


global q1q2_Start;
global q1q2_End;

% Initial Condition
l1 = 230.1390;
l2 = 147.7;
deg = pi/180;
PosOri = {0 0 0.208879343162506 0 [] [], pi/2 -pi/2};
n = 20;
Time = [0,5];
handles=guihandles();

addpath(genpath(pwd));

%% Input values [q0, q1, q2, q3, q4]

switch Mode
    case {1,2,3,4,5}
        q1q2_Start = [  0*pi/180,   0*pi/180,   45*pi/180,    0*pi/180,    45*pi/180];  
        q1q2_End = [ q0*pi/180,   q11*pi/180,   q12*pi/180,    q21*pi/180,    q23*pi/180];
      
    case {6,7,9}
        q1q2_Start = [  0*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   90*pi/180];
        q1q2_End = [ q0*pi/180,  q11*pi/180,   q12*pi/180,  q13*pi/180,   q23*pi/180]; 
        
    case 8
        q1q2_Start = [  0*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   90*pi/180];
        q1q2_End = [ q0*pi/180,  q11*pi/180,   q12*pi/180,  q21*pi/180,   q22*pi/180]; 
        
    case 10
        q1q2_Start = [  0*pi/180,  180*pi/180,    0*pi/180,   45*pi/180,   180*pi/180];
        q1q2_End = [  q0*pi/180,  q11*pi/180,   q12*pi/180,  q22*pi/180,   q14*pi/180]; 

    case 11
        q1q2_Start = [  0*pi/180,  45*pi/180,    0*pi/180,   0*pi/180,   180*pi/180];
        q1q2_End = [  q0*pi/180,  q11*pi/180,   q21*pi/180,  q22*pi/180,   q23*pi/180]; 
        
    case 12
        q1q2_Start = [  0*pi/180,  90*pi/180,   45*pi/180,  -45*pi/180,   45*pi/180];
        q1q2_End = [ q0*pi/180,  q11*pi/180,   q12*pi/180,  q21*pi/180,    q22*pi/180];
end


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

%%
%for i = 1:n
    switch Mode
        case 1
            %% --3T2R--
            q11q12q21q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 2
            %% --3T1R--
            q11q12q21q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 3
            %% --RCB_3T1R_SingularityA1C1--
            q11q12q21q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj3T1RSingularityA1C1 = RCB3T1RSingularityA1C1(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1.RCB_3T1R_SingularityA1C1_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 4
            %% --RCB_3T1R_SingularityA2C2--
            q11q12q21q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj3T1RSingularityA2C2 = RCB3T1RSingularityA2C2(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA2C2.RCB_3T1R_SingularityA2C2_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 5
            %% --RCB_3T1R_SingularityA1C1A2C2--
            q11q12q21q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj3T1RSingularityA1C1A2C2 = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q23, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1A2C2.RCB_3T1R_SingularityA1C1A2C2_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 6
            %% --2T2R--
            q11q12q14q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 7
            %% --RCB1T2RRotAroundPoint--
            q11q12q14q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj1T2RRotAroundPoint = RCB1T2RRotAroundPoint(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj1T2RRotAroundPoint.RCB_2T2Rsixbar_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 8
            %% --2T2RfiveBar--
            q11q12q14q22 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj2T2Rfivebar = RCB2T2Rfivebar(PosOri, q11q12q14q22 , l1, l2);
            [p, ~, q1q2] = obj2T2Rfivebar.RCB_2T2R_FiveBar_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 9
            %% --2T2RthreeBar--
            q11q12q14q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj2T2Rthreebar = RCB2T2Rthreebar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rthreebar.RCB_2T2R_ThreeBar_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 10
            %% --RCB2RserialA1C1--
            q11q12q22q14 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj2RserialA1C1 = RCB2RserialA1C1(PosOri, q11q12q22q14 , l1, l2);
            [p, ~, q1q2] = obj2RserialA1C1.RCB_2R_serialA1C1_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 11
            %% --RCB2RserialA2C2--
            q12q21q22q23 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            obj2RserialA2C2 = RCB2RserialA2C2(PosOri, q12q21q22q23 , l1, l2);
            [p, ~, q1q2] = obj2RserialA2C2.RCB_2R_serialA2C2_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
        case 12
            %% --RCB2RserialA2C2--
            q11q12q21q22 = [Ang_Intep(2,n), Ang_Intep(3,n), Ang_Intep(4,n), Ang_Intep(5,n)];
            objFixedSerialChain = RCBFixedSerialChain(PosOri, q11q12q21q22 , l1, l2);
            [p, ~, q1q2] = objFixedSerialChain.RCB_FixedSerialChain_FK;
            q0q1q2 = [Ang_Intep(1,n), q1q2];
    end
            ReconbotANI_GUI(q0q1q2);
%             q0 = q0q1q2(1);
%             q11 = q0q1q2(2);
%             q12 = q0q1q2(3)-pi/4;
%             q14 = q0q1q2(5)+pi/4;
%             q21 = q0q1q2(7);
%             q22 = q0q1q2(8)-pi/4;
%             q23 = q0q1q2(9)-pi/2;
%             Slide = 0;
%             LeftArmAngle = pi/6 * sin(i*pi/n);
%             RightArmAngle = pi/6 * sin(i*pi/n);
            
            %q0q1q2SlideLeftRightArm(i,:) = [q0, q11, q12, q14, q21, q22, q23, LeftArmAngle, RightArmAngle, Slide];% * 180/pi;
            
            %q0q1q2SlideLeftRightArm_time(i,:) = [time_Intep(i), q0, q11, q12, q21, q22, q23, LeftArmAngle, RightArmAngle, Slide];
%end
           
            
            
            