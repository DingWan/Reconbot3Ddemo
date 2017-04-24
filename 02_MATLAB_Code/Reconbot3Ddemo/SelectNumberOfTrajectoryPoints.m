

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
    str = {'3T2R';'3T1R';'3T1R-SingularityA1C1';'3T1R-SingularityA2C2';'3T1R-SingularityA1C1A2C2';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    [Mode,v] = listdlg('PromptString','Select a Operation Mode:','SelectionMode','single',...
        'ListString',str);
    if v == 0
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q0 = 0; 
    q0q1q2_0 = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_0 = [0 0 253.3124 0 0 0, 0 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ---------------------
    IK_p_ABC_q1q2;
    %%--------------------------
    
    %%
    if isreal(q1q2) == 0 || (WSvalue(1) == 0 && WSvalue(2) == 0 && WSvalue(3) == 0)
        errordlg('Inputs exceed workspace, Please Modify!','Workspace Error');
        while(WSvalue(1) == 0) 
            IK_p_ABC_q1q2;
        end
        %q1q2 = [0, pi/3, pi/3, pi/3, 0, 0, pi/3, pi/3, pi/3, 0];
    end
    
    %%------------------- Select appropriate q0q1q2 value -------------------
    %%----------- Input the number between 1~length(q1q2(:,1)) ------------    
    SolutionRow_cell = {};
    while(isempty(SolutionRow_cell))
        SolutionRow_cell = inputdlg({'Configurations,  RCBconfigurations:Input 0 quit!'},'NumTrajPoints', [1 40]);
        if isempty(SolutionRow_cell) == 0
            SolutionRow = str2num(SolutionRow_cell{1});
            if SolutionRow >= 1 && SolutionRow <= length(q1q2(:,1))
                %%------------ Plot two branch chain--------------
                q0q1q2_display = [0, q1q2(SolutionRow,:)];
                q0q1q2_all = [zeros(length(q1q2(:,1)),1), q1q2];
                ReconbotANI(q0q1q2_display);
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
    q0q1q2_current = [zeros(length(q1q2(:,1)),1), q1q2];
    SelectMode(NumTP) = Mode;    
    
    if NumTP == 1 % The starting step is always the singularity position in 3T1R mode
        Mode_Pos_Ori_TrajPoints_cell{1,1} = {5,{0, 0, 253.3124, 0, [], [], 0, 0},q0q1q2_0};     
    end
    
    if Mode == 10 || Mode == 11
        Mode_Pos_Ori_TrajPoints_cell{NumTP + 1,1} = {Mode, {po{:}},q0q1q2_all, Pos_Ori};
    else
        Mode_Pos_Ori_TrajPoints_cell{NumTP + 1,1} = {Mode, {po{:}},q0q1q2_all};
    end

end

%save('InsertRowTransiConfig_Test.mat')