clear
clc

% Initial Condition
l1 = 230.1390;
l2 = 147.7;
deg = pi/180;
PosOri = {0 0 208.879343162506 [] [] 0 , 0 0};
n = 20;
Time = [0,5];
q0q1q2 = [];
p_Output = [];

addpath(genpath(pwd));

%InitHome;
load('DemoMultiModes_6points.mat');

%% Input values [q0, q1, q2, q3, q4]
Mode = HomePos2SelectedEndPos_OutputData_Origin.Mode_det_Jq_Jc_J_mat(:,1);
Time = HomePos2SelectedEndPos_OutputData_Origin.Time_mat;
q012Ang = HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat;
q0_q11q12q14_q21q22q23_Ang = [q012Ang(:,1)  q012Ang(:,2)  q012Ang(:,3)  q012Ang(:,5)  q012Ang(:,7)  q012Ang(:,8)  q012Ang(:,9)];
Ang_Intep = q0_q11q12q14_q21q22q23_Ang;

%%
for i = 1:340 %length(Ang_Intep)
    if rem(i,n) == 0 && i < length(q012Ang(:,1)) && (Mode(i)<=5 || Mode(i+1)==12) && norm(q012Ang(i,:) - q012Ang(i+1,:)) < 1e-12
        i = i + 1;
        j = 1;
    else
        j = 0;
    end
    switch Mode(i)
        case 1
            %% --3T2R--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 2
            %% --3T1R--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
            [p, ~, q1q2] = obj3T1R.RCB_3T1R_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 3
            %% --RCB_3T1R_SingularityA1C1--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            obj3T1RSingularityA1C1 = RCB3T1RSingularityA1C1(PosOri, q11q12q21q22, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1.RCB_3T1R_SingularityA1C1_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 4
            %% --RCB_3T1R_SingularityA2C2--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            obj3T1RSingularityA2C2 = RCB3T1RSingularityA2C2(PosOri, q11q12q21q22, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA2C2.RCB_3T1R_SingularityA2C2_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 5
            %% --RCB_3T1R_SingularityA1C1A2C2--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            obj3T1RSingularityA1C1A2C2 = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q22, l1, l2);
            [p, ~, q1q2] = obj3T1RSingularityA1C1A2C2.RCB_3T1R_SingularityA1C1A2C2_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 6
            %% --2T2R--
            q11q12q14q23 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,4), Ang_Intep(i,7)];
            obj2T2Rsixbar = RCB2T2Rsixbar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 7
            %% --RCB1T2RRotAroundPoint--
            q11q12q14q23 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,4), Ang_Intep(i,7)];
            obj1T2RRotAroundPoint = RCB1T2RRotAroundPoint(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj1T2RRotAroundPoint.RCB_2T2Rsixbar_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 8
            %% --2T2RfiveBar--
            q11q12q14q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,4), Ang_Intep(i,6)];
            obj2T2Rfivebar = RCB2T2Rfivebar(PosOri, q11q12q14q22 , l1, l2);
            [p, ~, q1q2] = obj2T2Rfivebar.RCB_2T2R_FiveBar_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 9
            %% --2T2RthreeBar--
            q11q12q14q23 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,4), Ang_Intep(i,7)];
            obj2T2Rthreebar = RCB2T2Rthreebar(PosOri, q11q12q14q23 , l1, l2);
            [p, ~, q1q2] = obj2T2Rthreebar.RCB_2T2R_ThreeBar_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 10
            %% --RCB2RserialA1C1--
            q11q12q22q13 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,6), pi];
            obj2RserialA1C1 = RCB2RserialA1C1(PosOri, q11q12q22q13 , l1, l2);
            [p, ~, q1q2] = obj2RserialA1C1.RCB_2R_serialA1C1_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 11
            %% --RCB2RserialA2C2--
            q12q21q22q23 = [Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6), Ang_Intep(i,7)];
            obj2RserialA2C2 = RCB2RserialA2C2(PosOri, q12q21q22q23 , l1, l2);
            [p, ~, q1q2] = obj2RserialA2C2.RCB_2R_serialA2C2_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
        case 12
            %% --RCB2RserialA2C2--
            q11q12q21q22 = [Ang_Intep(i,2), Ang_Intep(i,3), Ang_Intep(i,5), Ang_Intep(i,6)];
            objFixedSerialChain = RCBFixedSerialChain(PosOri, q11q12q21q22 , l1, l2);
            [p, ~, q1q2] = objFixedSerialChain.RCB_FixedSerialChain_FK;
            %q0q1q2 = [Ang_Intep(i,1), q1q2];
    end
    
    q0q1q2_norm = [];
    p_norm = [];
    % Find the best configuration with input q1q2
    if i>1 && length(q1q2(:,1))>1 && isempty(q0q1q2) ~= 1
        % Find the suitable one solution
        for k = 1 : length(q1q2(:,1))
            if j == 1 && k == 1
                i = i - 1;
            end
            q0q1q2_Cur = [Ang_Intep(i,1), q1q2(k,:)];
            q0q1q2_norm(k) = norm(q0q1q2_Cur - q0q1q2(i-1,:));
        end
        [rowsq1q2,colsq1q2] = find(q0q1q2_norm == min(min(q0q1q2_norm)));
        SolutionRow_q1q2 = colsq1q2(1);
    else
        if j == 1
            i = i - 1;
        end
        colsq1q2 = 1;
    end
  
%     if i == 110
%       xx = 1    
%     end
    
    % Find the correct Posture 
    if i>1 && length(p(:,1))>1 && isempty(p_Output) ~= 1
        % Find the suitable one solution
        for k = 1 : length(p(:,1))
            p_Cur =  p(k,:);
            p_norm(k) = norm(p_Cur - p_Output(i-1,:));
        end
        [rowsp,colsp] = find(p_norm == min(min(p_norm)));
        SolutionRow_p = colsp(1);
    else
        colsp = 1;
    end
    
    p_Output(i,:) = [p(colsp(1),1:3), 180/pi*p(colsp(1),4:6)];
    q0q1q2(i,:) = [Ang_Intep(i,1), q1q2(colsq1q2(1),:)];
    %ReconbotANI(q0q1q2(i,:));
            
end

%%
figure(2)
plot3(p_Output(:,1),p_Output(:,2),p_Output(:,3),'b-')
grid on
axis equal

figure(3)
i = 1:1:340;
plot(Time(i),p_Output(i,4),'r-');hold on
plot(Time(i),p_Output(i,5),'g-');hold on
plot(Time(i),p_Output(i,6),'b-');hold on
grid on

figure(4)
i = 1:1:340;
plot(Time(i),p_Output(i,1),'r-');hold on
plot(Time(i),p_Output(i,2),'g-');hold on
plot(Time(i),p_Output(i,3),'b-');hold on
grid on            