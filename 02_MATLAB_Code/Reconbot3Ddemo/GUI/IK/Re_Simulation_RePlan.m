    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%------------------------------------------Re_Plan-------------------------------------------%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

%     % =============== Assign Initial Values =============
         q0q1q2_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat;
    q11q12q14_q21q22q23_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.q11q12q14_q21q22q23;
             MP_Pos_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Pos_mat;
               Time_mat_NewAdjust = HomePos2SelectedEndPos_OutputData_RePlan.Time_mat;
               
               
    %% =============== Plot joint Angles and Check the correntness of Joint Velocity Limitation ==================
    for OnlyUsedforFolding_ValueCheck_PlotValue = 1:1
        PlotAngle_NewAdjust;
    end