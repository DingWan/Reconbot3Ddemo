
% l1 = 230.1390;
% l2 = 147.7;

%%
% Pos_Intep = [];
% Vel_Intep = []; 
% Acc_Intep = []; 
% time_Intep = [];
% %
% q0q1q2_Pos_mat = [];
% q0q1q2_Vel_mat = [];
% q0q1q2_Acc_mat = [];
% %
% MP_Pos_Intep = [];
% MP_Vel_Intep = [];
% MP_Acc_Intep = [];
% MP_time_Intep = [];
% %
% q0q1q2_P2P_Pos_Intep = [];
% q0q1q2_P2P_Vel_Intep = [];
% q0q1q2_P2P_Acc_Intep = [];
% q0q1q2_P2P_time_Intep = [];
% %
% PosOri_Output_mat = [];
%
col = []; % Number and position of 'Self_adjustment_Enable_Disable == 1'
col_Self_adjustment = [];


q0q1q2_FixedMode = [0, 0, 0, pi, -pi/2, 0, 0, 0, pi, -pi/2, 0];
q0q1q2_RectagularPosition = [0, 0, 0, pi/2, 0, 0, 0, pi/2, pi/2, -pi/2, 0];
Posori_Initial = [0 0 0 0 0 0, 0 0];
Posori_RectagularPosition = [0 147.7 147.7 0 0 0, 0 0]; 

angle_previous = 0;
angle_current = pi/2;

n = NumIntepoPoints;
Time = [0 5];
%% 5-Grade Polynomial Intepotation
PO = [angle_previous,angle_current];
v = [0, 0];
a = [0, 0];

[ a0,a1,a2,a3,a4,a5,T ] =  PTP_Polynom5(PO, v, a, Time) ;

t = linspace(0,T,n);
px = [a5,a4,a3,a2,a1,a0];
%px = [a0,a1,a2,a3,a4,a5];
pxd = polyder(px);
pxdd = polyder(pxd);
Ang = polyval(px,t);
Vel = polyval(pxd,t);
Acc = polyval(pxdd,t);
time_Intep = t + Time(1)*ones(1,n);

Ang_Intep(1,:) = Ang;
Vel_Intep(1,:) = Vel;
Acc_Intep(1,:) = Acc;

Pos_Intep(:,1) = zeros(n,1);
Pos_Intep(:,2) = l2 * (1-cos(Ang_Intep));
Pos_Intep(:,3) = l2 * sin(Ang_Intep);  

% figure(2)
% plot3(Pos_Intep(:,1),Pos_Intep(:,2),Pos_Intep(:,3),'r-')
% grid on
% axis equal

%%
Mode = 1;
PosOri = {0 0 0 0 [] [], 0 0};
q0 = 0;
q11 = 0;
q21 = 0;
Time_inteval = 5;
MP_time_Intep = time_Intep' + 5*ones(n,1);
q0q1q2_OptimalRow = q0q1q2_FixedMode;

for i = 1:n

PosOri = {Pos_Intep(i,1)  Pos_Intep(i,2)  Pos_Intep(i,3) 0 [] [], 0 0};
[ p, ~, ~, q1q2, ~ ] = IK(Mode, PosOri, q0, q11, q21, q0q1q2_FixedMode, l1, l2);
q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];

% Optimal Joints Solution
q1q2_matrix_norm = [];
for k = 1:length(q0q1q2_CurrentStep(:,1))
    if i == 1
        q1q2_matrix_norm(k) = norm(q0q1q2_CurrentStep(k,2:11) - q0q1q2_OptimalRow(i,2:11));
    else
        q1q2_matrix_norm(k) = norm(q0q1q2_CurrentStep(k,2:11) - q0q1q2_OptimalRow(i-1,2:11));
    end
end

[rowsq1q2,colsq1q2] = find(q1q2_matrix_norm == min(min(q1q2_matrix_norm)));
SolutionRow_q1q2 = colsq1q2(1);
q0q1q2_OptimalRow(i,:) = [q0, q0q1q2_CurrentStep(SolutionRow_q1q2,2:11)];

%ReconbotANI(q0q1q2_OptimalRow(i,:)); 
end

PosOri_Output_Intep = Pos_Intep;
q0q1q2_P2P_Pos_Intep = q0q1q2_OptimalRow;

% Position, Velocity, Acceleration Calcuation of joint angles
% Velocity Calcuation
i = 1;
for j = 1:n
    if j == 1 || j == n
        q0q1q2_P2P_Vel_Intep((i-1)*n + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
    else
        q0q1q2_P2P_Vel_Intep((i-1)*n + j,:) = ...
            ( q0q1q2_P2P_Pos_Intep((i-1)*n + j + 1,:) - q0q1q2_P2P_Pos_Intep((i-1)*n + j,:) ) / ...
            (       MP_time_Intep((i-1)*n + j + 1, 1) - MP_time_Intep((i-1)*n + j, 1)       );
    end
end

% Acceleration Calcuation
for j = 1:n
    if j == 1 || j == n
        q0q1q2_P2P_Acc_Intep((i-1)*n + j,:) =  zeros(1,length(q0q1q2_P2P_Pos_Intep(1,:)));
    else
        q0q1q2_P2P_Acc_Intep((i-1)*n + j,:) = ...
            ( q0q1q2_P2P_Vel_Intep((i-1)*n + j + 1,:) - q0q1q2_P2P_Vel_Intep((i-1)*n + j,:)  ) / ...
            (       MP_time_Intep((i-1)*n + j + 1, 1) - MP_time_Intep((i-1)*n + j, 1)        );
    end
end

% Moving Platform Position
    PosOri_Output_mat = [PosOri_Output_mat; PosOri_Output_Intep];

% Position, Velocity, Acceleration Calcuation of joint angles                                                                                                       NumIntepoPoints, Start_Time, Time_inteval, l1, l2);
    q0q1q2_Pos_mat = [ q0q1q2_Pos_mat; q0q1q2_P2P_Pos_Intep ];
    q0q1q2_Vel_mat = [ q0q1q2_Vel_mat; q0q1q2_P2P_Vel_Intep ];
    q0q1q2_Acc_mat = [ q0q1q2_Acc_mat; q0q1q2_P2P_Acc_Intep ];
% Time
    Time_mat = [ Time_mat; MP_time_Intep ];
   
%%
% Time_mat = MP_time_Intep;
% Mode_det_Jq_Jc_J_mat(:,1) = zeros(n,1);
% 
% Initial2Rectagular_q1q2 = [ q0q1q2_Pos_mat(:,2), q0q1q2_Vel_mat(:,2), q0q1q2_Acc_mat(:,2),...
%                             q0q1q2_Pos_mat(:,3), q0q1q2_Vel_mat(:,3), q0q1q2_Acc_mat(:,3),...
%                             q0q1q2_Pos_mat(:,5), q0q1q2_Vel_mat(:,5), q0q1q2_Acc_mat(:,5),...
%                             q0q1q2_Pos_mat(:,7), q0q1q2_Vel_mat(:,7), q0q1q2_Acc_mat(:,7),...
%                             q0q1q2_Pos_mat(:,8), q0q1q2_Vel_mat(:,8), q0q1q2_Acc_mat(:,8),...
%                             -q0q1q2_Pos_mat(:,9), -q0q1q2_Vel_mat(:,9), -q0q1q2_Acc_mat(:,9),...
%                             q0q1q2_Pos_mat(:,1), q0q1q2_Vel_mat(:,1), q0q1q2_Acc_mat(:,1),...
%                             Time_mat(:,1), Mode_det_Jq_Jc_J_mat(:,1)
%                           ];


