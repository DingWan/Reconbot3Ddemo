%
%clf
clc
clear

L1 = 0.2301390;
L2 = 0.1477;
l1 = 0.2301390;
l2 = 0.1477;
deg = pi/180;
addpath(genpath(pwd)); % Enalbe all folders
q0q1q2_HomePosition = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
PosOri = [0 0 0.208879343162506 0 0 0, 0 0];

%% Calculate the IK Jacobian


%%
po_start = { 0.100 0.100 0.20  1*pi/180 [] []};
  po_end = { 0.100 0.100 0.20 30*pi/180 [] []};
Time = [0 5];
n = 20;
%% 5-Grade Polynomial Intepotation
for i = 1: 4
PO = [po_start{i}, po_end{i}];
v = [0, 0];
a = [0, 0];

[ a0,a1,a2,a3,a4,a5,T ] =  PTP_Polynom5(PO, v, a, Time) ;

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

%%
tic
q11q12q21q22 = [];
for i = 1:n
    
    
    po = {Pos_Intep(1,i), Pos_Intep(2,i), Pos_Intep(3,i), Pos_Intep(4,i), [], []};    
    %while norm(err)>0.001        
       %% ============================ IK ==============================
        obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
        [p, ~, ~, q1q2_all, ~] = obj3T1R.RCB_3T1R_IK;
        if i == 1
            for j = 1:length(q1q2_all(:,1))
                q1_matrix_norm(j) = norm(q1q2_all(j,1:5) - q0q1q2_HomePosition(1,2:6));
                q2_matrix_norm(j) = norm(q1q2_all(j,6:10) - q0q1q2_HomePosition(1,7:11));
            end
        else
            for j = 1:length(q1q2_all(:,1))
                q1_matrix_norm(j) = norm(q1q2_all(j,1:5) - q1q2(i-1,1:5));
                q2_matrix_norm(j) = norm(q1q2_all(j,6:10) - q1q2(i-1,6:10));
            end
        end  
        [rowsq1,colsq1] = find(q1_matrix_norm == min(min(q1_matrix_norm)));
        [rowsq2,colsq2] = find(q2_matrix_norm == min(min(q2_matrix_norm)));
        SolutionRow_q1 = colsq1(1);
        SolutionRow_q2 = colsq2(1);        
        q1q2(i,:) = [q1q2_all(SolutionRow_q1,1:5), q1q2_all(SolutionRow_q2,6:10)];
        
        % initial Position        
        q11 = q1q2(i,1); q12 = q1q2(i,2); q13 = q1q2(i,3); q14 = q1q2(i,4); q15 = q1q2(i,5);
        q21 = q1q2(i,6); q22 = q1q2(i,7); q23 = q1q2(i,8); q24 = q1q2(i,9); q25 = q1q2(i,10);
        q0q1q2(i,:) = [0, q1q2(i,:)];
        % InitHome;
         ReconbotANI(q0q1q2(i,:));
        % ===============================================================
        
        % Jacobian Matrix
        Enable_Mode_JacoMat = 2;
        UnifiedJacobianMatrix_ScrewTheory_DimenstionalHomogeneous;
        ConditionNumber(i) = max(max(S))/min(max(S));        
        
        det_Jq1_Ob_3T1R(i) = det(Jq1_Ob_3T1R); % normized  /norm(Jq1_Ob_3T1R)
        det_J_Ob_3T1R(i) = det(J_Ob_3T1R);
        det_Jc_Ob_3T1R(i) = det(J_Ob_3T1R(5:6,1:2)/norm(J_Ob_3T1R(5:6,1:2))); % normized
        
end
toc
%%
figure(3)
[~,col_q1] = find(abs(det_Jq1_Ob_3T1R) == max(abs(det_Jq1_Ob_3T1R)));
[~,col_Ob] = find(abs(det_J_Ob_3T1R) == max(abs(det_J_Ob_3T1R)));
det_Jq1_Ob_3T1R = det_Jq1_Ob_3T1R/abs(det_Jq1_Ob_3T1R(col_q1));
det_J_Ob_3T1R = det_J_Ob_3T1R/abs(det_J_Ob_3T1R(col_Ob));
i = 1:n;
plot(i,det_Jq1_Ob_3T1R,'r-');hold on % with redundant actuation: det_Jq1_Ob_3T1R<0.2
plot(i,det_Jc_Ob_3T1R,'g-');hold on
plot(i,det_J_Ob_3T1R,'b-');hold on
grid on

figure(4)
plot(i,ConditionNumber,'r-');hold on
grid on