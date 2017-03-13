%%--------------------2-RER Mode selection-----------------------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 15 Nov 2016.
%     The copyright is belong to Wan Ding.

%       po = {cell}               Position and rotation of the MP, cell
%       po_cell = {cell}          Position and rotation of the MP, cell
%       po_num = {num}            Position and rotation of the MP, number
%       SolutionRow               The row of solution in 8 solutions.
%       EulerAngle_q11_theta      Euler Angle(Z-Y-X)  q11, and theta: [alpha, beta, gamma, q11, theta]

%%-------------------------------------------------------------------
clc
clear
%clf

l1 = 2.2;
l2 = 1.4725;
deg = pi/180;
n = 25;
Tension=0;

addpath(genpath(pwd)); % Enalbe all folders inside "SSoop"
InitHome;       
% hold on

%% Select the number of trajectory points
NumTrajPoints_cell = {};
while(isempty(NumTrajPoints_cell))
    NumTrajPoints_cell = inputdlg({'Number of Trajectory Points'},'NumTrajPoints', [1 40]);
end
NumTrajPoints_num = str2num(NumTrajPoints_cell{1});

%% Choose the configuration of each trajectory point
for NumTP = 1:NumTrajPoints_num
    %%
    addpath(genpath(pwd)); % Enalbe folder "RCB_3D_demo"
    str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    [s,v] = listdlg('PromptString','Select a Operation Mode:','SelectionMode','single',...
        'ListString',str);
    if v == 0
        break;
    end
    
    %% ---------------------
    IK_p_ABC_q1q2;
    %%--------------------------
    
    %%
    if isreal(q1q2) == 0
        errordlg('Inputs exceed workspace, Please Modify!','Workspace Error');
        q1q2 = [0, pi/3, pi/3, pi/3, 0, 0, pi/3, pi/3, pi/3, 0];
    end
    
    %%------------------- Select appropriate q0q1q2 value -------------------
    %%----------- Input the number between 1~length(q1q2(:,1)) ------------    
    SolutionRow_cell = {};
    while(isempty(SolutionRow_cell))
        SolutionRow_cell = inputdlg({'RCBconfig (1~16/32):Input 0 quit!'},'NumTrajPoints', [1 40]);
        if isempty(SolutionRow_cell) == 0
            SolutionRow = str2num(SolutionRow_cell{1});
            if SolutionRow >= 1 && SolutionRow <= length(q1q2(:,1))
                %%------------ Plot two branch chain--------------                
                q0q1q2 = [0, q1q2(SolutionRow,:)];
                ReconbotANI(q0q1q2);
                %--------------------------------------------------
                SolutionRow_cell = {};
                p = (ABC(SolutionRow,7:9) + ABC(SolutionRow,16:18))/2;
            else
                SolutionRow_cell = {0};
            end
        else
            continue;
        end
    end
    
    Pos_Ori = [p, EulerAngle_q11_theta(1,1:3)];
    q0q1q2_TransitionStrategy(NumTP,:) = q0q1q2;
    s_SelectedMode(NumTP) = s;
    
    % Trajectory point and the selected operational mode in each step 
    Mode_Pos_Ori_TrajPoints_Martix(NumTP,:) = [s, Pos_Ori];
    if NumTP == 1 % The starting step is always the singularity position in 3T1R mode
        Mode_Pos_Ori_TrajPoints_cell{NumTP,1} = {0, 0, 255.0445, 0, [], [], 0, 0};
        TrajPointsAndOperationalModes(NumTP,:) = [1, [0 0 255.0445 0 0 0], [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0]];
    end
    Mode_Pos_Ori_TrajPoints_cell{NumTP + 1,1} = po;
    % Selected modes, trajectory points, and possible estimate value for
    % each step with out interference.
    TrajPointsAndOperationalModes(NumTP + 1,:) = [s, Pos_Ori, q0q1q2];
    
end

save('InsertRowTransiConfig_Test.mat')

% load('InsertRowTransiConfig_Test.mat')

%% Transition Strategy
MPOTP_cell = TransitionStrategy(TrajPointsAndOperationalModes, Mode_Pos_Ori_TrajPoints_Martix, Mode_Pos_Ori_TrajPoints_cell, NumTP);  


%% Value of q0q1q2 = [q0, q11-q15, q21-q25], PosOri = [Px, Py, Pz, alpha, beta, gamma]
tic
for IntepPointNum = 1 : length(MPOTP_cell)
        
    po = MPOTP_cell{IntepPointNum}; % 3T1R mode - self-adjustment
    
    q0 = 0;
    
    for i = 1:6
        if isempty(po{i}) == 1
            p_BinaryCode(i) = 0;
        else
            p_BinaryCode(i) = 1;
        end
    end
    
    %%----------------------- Calculate RCB IK -------------------------
    if isequal(p_BinaryCode(1:6), [1 1 1 1 0 0]) == 1
        % 3T1R mode:  [1 1 1 1 0 0]
        % p = [x, y, z, alpha, [], []];
        q11q12q21q22 = [];
        obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
        [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;   
    elseif isequal(p_BinaryCode, [1 1 1 0 0 1]) == 1
        % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
        % p = [x, y, z, [], [], gamma]
        q11q12q14q23 = [];
        obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
        [p, EulerAngle_q11_theta, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
    elseif isequal(p_BinaryCode, [1 1 1 0 1 1]) == 1
        % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
        % p = [x, 0, z, [], beta, 0]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_2T2R_FiveBar_IK(po, l1, l2);
    elseif isequal(p_BinaryCode, [1 1 1 1 1 0]) == 1
        % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
        % p = [x, 0, z, 0, beta, []]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_ThreeBar_IK(po, l1, l2);
    elseif isequal(p_BinaryCode, [0 0 1 0 1 1]) == 1
        % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
        % p = [[], [], z, [], beta, gamma]; x = y = 0
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(po, l1, l2);
    elseif isequal(p_BinaryCode, [1 1 0 0 0 0]) == 1
        if po{2} < 0
            % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y < 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(po, l1, l2);
        elseif po{2} > 0
            % Four-bar linkage with Serial Chain A2C2: [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y > 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA2C2_IK(po, l1, l2);
        end
    elseif isequal(p_BinaryCode, [1 1 1 0 0 0]) == 1
        % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
        % p = [0, 0, 0, [], [], []]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
    end
    %%--------------------------- Method: End -----------------------------
    p_cell{IntepPointNum,:} = p;
    EulerAngle_q11_theta_cell{IntepPointNum,:} = EulerAngle_q11_theta;
    ABC_cell{IntepPointNum,:} = ABC;    
    
    q0q1q2_EachTrajPoint = [zeros(length(q1q2(:,1)),1), q1q2];
    q0q1q2_cell{IntepPointNum,:} = q0q1q2_EachTrajPoint;
end
toc

%% Motion planning
tic
for IntepPointNum = 1 : length(MPOTP_cell)
    
    q0q1q2 = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0];
    p_0 = [0 0 255.0445 0 0 0, 0 0];
    
    MotionPlanningOptimalSoultion;
    
    if length(MPOTP_cell{IntepPointNum}) == 8 && length(MPOTP_cell{IntepPointNum+1}) == 6 
        
    elseif length(MPOTP_cell{IntepPointNum-1}) == 8 && length(MPOTP_cell{IntepPointNum}) == 6 % IntepPointNum > 2
        q0q1q2_mat(n*(IntepPointNum-2)+1,:) = q0q1q2_mat(n*(IntepPointNum-2)+2,:); 
        q0q1q2_mat([n*(IntepPointNum-2)+1:n*(IntepPointNum-1), n*(IntepPointNum-1)+1:n*IntepPointNum],:)...
            = q0q1q2_mat([n*(IntepPointNum-1)+1:n*IntepPointNum, n*(IntepPointNum-2)+1:n*(IntepPointNum-1)],:);
    else 
        
    end

end
toc

save('q0q1q2_mat.mat')
%% 3D Animation
for i = 1:length(q0q1q2_mat)
    %========================== Animation ============================
    ReconbotANI(q0q1q2_mat(i,:));
    set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
    %============================ End ================================
end

clr_trail_CollisionPoints_button_press;