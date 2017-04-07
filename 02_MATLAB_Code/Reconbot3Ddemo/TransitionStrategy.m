
%%----------2-RER Transition strategy among different modes----------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 13 Dec 2016.
%     Version 1.1

%%-------------------------------------------------------------------
% TrajPointsAndOperationalModes(NumTP,:) = [s, Pos_Ori, q0q1q2];

function  [ MPOTP_cell, Self_adjustment_Enable_Disable] = TransitionStrategy(Mode_previous,Posture_previous, q0q1q2_previous, Mode_current,Posture_current)

%%---------2-RER Transition strategy among different modes-----------
%%-------------------------------------------------------------------
%
%     This is written by Wan Ding in 13 Dec 2016.
%
%       NumberofInsertRows                             Number of Insert rows between two different modes
%       RowSequenceoftheFirstInsertRow                 The row sequence of the first insert row
%       InsertRowInfos(IntepPointSeq,:) = [];          InsertRowInfos(IntepPointSeqence,:) 
%                                                                = [NumberofInsertRows, RowSequenceoftheFirstInsertRow];
%       TPOM                                           Trajectory Points And Operational Modes
%       MPOTP_Mat                                      Mode_Position_Orientation_TrajectoryPoints_Matrix 
%       MPOTP_cell                                     Mode_Position_Orientation_TrajectoryPoints_cell
%       Self_adjustment_Enable_Disable = 0/1/2         0: without adjustment; 1: from step 3/4/5 to other modes; 2 from other mode to step 3/4/5
%%-------------------------------------------------------------------

%MPOTP_cell{1} = {Mode_previous,Posture_previous};
MPOTP_cell{1,:} = {Mode_current,Posture_current};
% 
Self_adjustment_Enable_Disable = 0;
NumberofInsertRows = 0;
RowSequenceoftheFirstInsertRow = 0;
InsertRowInfos = [0, 0];
   
    %%-------------Step 1st: Calculate the SplinePoints: i and i+1 ----------------------
    % str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    
    %%-------------Step 2nd: choose spline times: 1 or 2(mode change) -------------------
        switch Mode_previous
            case 1 % 3T2R
                switch Mode_current
                    case 1 % 3T2R
                        Self_adjustment_Enable_Disable = 0;
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {Mode_current, TPOM(IntepPointSeq + 1,2), TPOM(IntepPointSeq + 1,3), TPOM(IntepPointSeq + 1, 4), TPOM(IntepPointSeq + 1, 5), [], [], 0, []};
                        InsertRow_q0q1q2 = {0};
                        NumberofInsertRows = 1;
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 2;
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0)
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end
            
            case 2 % 3T1R
                switch Mode_current
                    
                    case 1 % 3T2R 

                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2                        
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 2;
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,1), TPOM(IntepPointSeq,2), TPOM(IntepPointSeq,3), TPOM(IntepPointSeq,4), 0, [], []};
                        InsertRow_q0q1q2 = {0};
                        NumberofInsertRows = 1;
                    case 7 % 2T2R-6-Bar(xy=0)
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,1), 0, 0, TPOM(IntepPointSeq + 1,4), 0, [], [], 0, 0};
                        [EulerAngle_q11_theta] = EulerAngles_beta_gamma_q11_IK(TPOM(IntepPointSeq + 1,5), TPOM(IntepPointSeq + 1,6), []);
                        InsertRow_TransiConfig{2,:} = {TPOM(IntepPointSeq,1), 0, 0, TPOM(IntepPointSeq + 1,4), 0, [], [], EulerAngle_q11_theta(4), EulerAngle_q11_theta(4)};                        
                        NumberofInsertRows = 2;
                    case 8 % 2T2R-5-Bar
                        InsertRow_TransiConfig = {Mode_current, TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig = {Mode_current, TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 10 % 2R-SerialA1C1
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,1), 0, 0, TPOM(IntepPointSeq, 4), 0, [], [], 0, 0};
                        NumberofInsertRows = 1;
                    case 11 % 2R-SerialA2C2
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,1), 0, 0, TPOM(IntepPointSeq, 4), 0, [], [], 0, 0};
                        NumberofInsertRows = 1;                        
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,1), 0, 0, 0, 0, [], []};
                        NumberofInsertRows = 1;                        
                end
            case 3 % 3T1R-SingularityA1C1
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 2;
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end  
                
            case 4 % 3T1R-SingularityA2C2
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2 
                        
                    case 5 % 3T1R-SingularityA1C1A2C2  
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 2;
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end
                
            case 5 % 3T1R-SingularityA1C1A2C2
                switch Mode_current
                    case 1 % 3T2R
                        %We need to intepolate 3D space
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 1;
                    case 2 % 3T1R
                        %We need to intepolate 3D space
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 1;                        
                    case 3 % 3T1R-SingularityA1C1
                        %We need to intepolate on a cylinder surface 
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 1; 
                    case 4 % 3T1R-SingularityA2C2
                         %We need to intepolate on a cylinder surface 
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        NumberofInsertRows = 1;      
                        Self_adjustment_Enable_Disable = 1;  
                    case 5 % 3T1R-SingularityA1C1A2C2 
                         %We need to intepolate on a line along z-axis
                         Self_adjustment_Enable_Disable = 0;
                    case 6 % 2T2R-6-Bar
                        %We need to intepolate 3D space                        
%                         InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,2), TPOM(IntepPointSeq,3), TPOM(IntepPointSeq,4), [], [], 0};
%                         InsertRow_q0q1q2 = {0};
%                         NumberofInsertRows = 1;  
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                         %We need to intepolate on o-xz plane
                        
                    case 9 % 2T1R-3-BarSerial
                        %We need to intepolate on o-xz plane
                        
                    case 10 % 2R-SerialA1C1
                        %We need to intepolate on shpere surface
                        
                    case 11 % 2R-SerialA2C2
                        %We need to intepolate on shpere surface
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end   
                
            case 6 % 2T2R-6-Bar
                
                switch Mode_current
                    case 1 % 3T2R
                        
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,2), TPOM(IntepPointSeq,3), TPOM(IntepPointSeq,4), [], [], 0};
                        InsertRow_q0q1q2 = {0};
                        NumberofInsertRows = 1;    
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end 
                
            case 7 % 2T2R-6-Bar(xy=0)
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 8 % 2T2R-5-Bar                
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 9 % 2T1R-3-BarSerial
                
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end 
                
            case 10 % 2R-SerialA1C1                
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 11 % 2R-SerialA2C2
                
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                end 
                
            case 12 % Fixed-SerialA1C1A2C2                
                switch Mode_current
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                end 
                
        end
        if NumberofInsertRows ~= 0
            for i = 1:NumberofInsertRows                
                % Insert the transition configurations in the original cell: Mode_Pos_Ori_TrajPoints_cell
                MPOTP_cell{i+1,:} = InsertRow_TransiConfig{i,:};
            end
            
        end   
end
