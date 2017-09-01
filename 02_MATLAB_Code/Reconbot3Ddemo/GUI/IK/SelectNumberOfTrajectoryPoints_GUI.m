
%% Choose the configuration of each trajectory point

    addpath(genpath(pwd)); % Enalbe folder "GUI_Reconbot"
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q0 = 0; 
    q0q1q2_0 = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_0 = [0 0 208.879343162506 0 0 0, 0 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ---------------------
    IK_ModeSelection_GUI;
    %%--------------------------
    
    Pos_Ori = [p, EulerAngle_q11_theta(1,1:3)];
    q0q1q2_current = [zeros(length(q1q2(:,1)),1), q1q2];
    SelectMode(NumTP) = Mode;    
    
    if NumTP == 1 % The starting step is always the singularity position in 3T1R mode
        Mode_Pos_Ori_TrajPoints_cell{1,1} = {5,{0, 0, 208.879343162506, 0, [], [], 0, 0},q0q1q2_0};     
    end
    
    if Mode == 10 || Mode == 11
        Mode_Pos_Ori_TrajPoints_cell{NumTP + 1,1} = {Mode, {po{:}},q0q1q2_all, Pos_Ori};
    else
        Mode_Pos_Ori_TrajPoints_cell{NumTP + 1,1} = {Mode, {po{:}},q0q1q2_all};
    end


%save('InsertRowTransiConfig_Test.mat')