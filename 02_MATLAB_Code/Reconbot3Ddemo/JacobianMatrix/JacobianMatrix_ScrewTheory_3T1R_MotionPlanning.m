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

PosOri = [0 0 0.1477 0 0 0, 0 0];
tic
%% Calculate the IK Jacobian
% -----------3T1R-------------
    po = { 0.010 0.0100 0.23 -1*pi/180 [] []};
    
    q11q12q21q22 = [];
    deltapo = 0.000001;
    Num_po_variables = 4;
    delta_po = deltapo * eye(Num_po_variables);
    J_dq_dx_eul = zeros( Num_po_variables, 6 );
    % --3T1R--
    % Previous
    obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
    [p_previous, ~, ~, q1q2_all, ~] = obj3T1R.RCB_3T1R_IK;
    q1q2_previous = q1q2_all(1,:);
    q1q2 = q1q2_previous;
    for i = 1:Num_po_variables
    % Current
    po_delta_po = po;
    po_delta_po{i} = po{i} + deltapo;
    obj3T1R = RCB3T1R(po_delta_po, q11q12q21q22, l1, l2);
    [p_current, EulerAngle_q11_theta, ~, q1q2_all, ~] = obj3T1R.RCB_3T1R_IK;
    q1q2_current = q1q2_all(1,:);
    dq(:,i) = (q1q2_current - q1q2_previous)';
    J_dx2dq_eul(:,i) = [  dq(1,i) / deltapo; %This whole thing is a single column
                          dq(2,i) / deltapo;
                          %dq(4,i) / deltapo;
                          dq(6,i) / deltapo;
                          dq(7,i) / deltapo;
                          %dq(8,i) / deltapo;
                        ];
    end
    %[U,S,V] = svd(J_dx2dq_eul)
    det(J_dx2dq_eul)
    % delta_q11q12q21q22
    delta_q1q2 = q1q2_current - q1q2_previous;
    q11q12q21q22_previous = [ q1q2_previous(1:2), q1q2_previous(6:7)];
    q11q12q21q22_current = [ q1q2_current(1:2), q1q2_current(6:7)];
    delta_q11q12q21q22 = [delta_q1q2(1:2), delta_q1q2(6:7)];
toc
 
%%
po_start = { 0.10 0.100 0.23 -30*pi/180 [] []};
po_end = { -0.10 0.100 0.15 10*pi/180 [] []};
Time = [0 5];
n = 200;
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
for i = 1:n
    
%     if i == 1
%         POstart = po_start;
%         POend = {Pos_Intep(1,1), Pos_Intep(2,1), Pos_Intep(3,1), Pos_Intep(4,1), [], []};
%         e = POend - POstart; 
%     else
%         POstart =  {Pos_Intep(1,i-1), Pos_Intep(2,i-1), Pos_Intep(3,i-1), Pos_Intep(4,i-1), [], []};
%         POend =  {Pos_Intep(1,i), Pos_Intep(2,i), Pos_Intep(3,i), Pos_Intep(4,i), [], []};
%         e = POend - POstart;        
%     end
%     err = cell2mat(POend) - cell2mat(POstart);    
    
    po = {Pos_Intep(1,i), Pos_Intep(2,i), Pos_Intep(3,i), Pos_Intep(4,i), [], []};    
    %while norm(err)>0.001        
       %% ============================ IK ==============================
        obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
        [p_previous, ~, ~, q1q2_all, ~] = obj3T1R.RCB_3T1R_IK;
        q1q2_previous = q1q2_all(1,:);
        q1q2 = q1q2_previous;
        % initial Position
        q11 = q1q2(1); q12 = q1q2(2); q13 = q1q2(3); q14 = q1q2(4); q15 = q1q2(5);
        q21 = q1q2(6); q22 = q1q2(7); q23 = q1q2(8); q24 = q1q2(9); q25 = q1q2(10);
        q0q1q2 = [0, q1q2];
        % InitHome;
        % ReconbotANI(q0q1q2);
        % ===============================================================
        
        % Jacobian Matrix
        UnifiedJacobianMatrix_ScrewTheory;
        
        det_Jq1_Ob_3T1R(i) = det(Jq1_Ob_3T1R/norm(Jq1_Ob_3T1R)); % normized
        det_J_Ob_3T1R(i) = det(J_Ob_3T1R);
        det_Jc_Ob_3T1R(i) = det(J_Ob_3T1R(5:6,1:2)/norm(J_Ob_3T1R(5:6,1:2))); % normized
        
        %q11q12q21q22_current_J = [ q11q12q21q22_previous 0 0 ] + (J_Ob_3T1R * [ 0, 0, deltapo, deltapo, deltapo, deltapo ]')';
        
%         qsdot = pinv(Jacob) * transpose(err);
%         delta_q =  qsdot;
%         q = q + factor * transpose(delta_q*delta_t);
%         Jacob = PR9.jacob0(q,'eul');
%         FTP = PR9.fkine(q);
%         P =  [transpose(FTP(1:3,4)),tr2eul(FTP)];
%         e = POend - P;
   % end
end
toc
%%
% [U,S,~] = svd(J_Ob_3T1R )
% [U,S,~] = svd(J_dx2dq_eul)
