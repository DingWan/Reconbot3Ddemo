
%%----------2-RER Transition strategy among different modes----------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 13 Dec 2016.
%     The copyright is belong to Wan Ding and IGM.

%%-------------------------------------------------------------------
% TrajPointsAndOperationalModes(NumTP,:) = [s, Pos_Ori, q0q1q2];

function  MPOTP_cell = TransitionStrategy(TPOM, MPOTP_Mat, MPOTP_cell, NumTP)

%%---------2-RER Transition strategy among different modes-----------
%%-------------------------------------------------------------------
%
%     This is written by Wan Ding in 13 Dec 2016.
%     The copyright is belong to Wan Ding and IGM.
%
%       NumberofInsertRows                             Number of Insert rows between two different modes
%       RowSequenceoftheFirstInsertRow                 The row sequence of the first insert row
%       InsertRowInfos(IntepPointSeq,:) = [];          InsertRowInfos(IntepPointSeqence,:) 
%                                                                = [NumberofInsertRows, RowSequenceoftheFirstInsertRow];
%       TPOM                                           Trajectory Points And Operational Modes
%       MPOTP_Mat                                      Mode_Position_Orientation_TrajectoryPoints_Matrix 
%       MPOTP_cell                                     Mode_Position_Orientation_TrajectoryPoints_cell
%       IntepPointSeq                                  Intepotation Point Seqence
%
%%-------------------------------------------------------------------

MPOTP_InsertRow = MPOTP_Mat;

NumberofInsertRows = 0;
RowSequenceoftheFirstInsertRow = 0;
InsertRowInfos = [0, 0];

for IntepPointSeq = 1 : NumTP 
    
    %%-------------Step 1st: Calculate the SplinePoints: i and i+1 ----------------------
    % str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    
    
    %%-------------Step 2nd: choose spline times: 1 or 2(mode change) -------------------
    if TPOM(IntepPointSeq,1) == TPOM(IntepPointSeq+1,1)
        continue;
    else
        switch TPOM(IntepPointSeq,1)
            
            case 1 % 3T1R
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    
                    case 2 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,2), TPOM(IntepPointSeq,3), TPOM(IntepPointSeq,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 3 % 2T2R-6-Bar(xy=0)
                        InsertRow_TransiConfig{1,:} = {0, 0, TPOM(IntepPointSeq + 1,4), 0, [], [], 0, 0};
                        [EulerAngle_q11_theta] = EulerAngles_beta_gamma_q11_IK(TPOM(IntepPointSeq + 1,5), TPOM(IntepPointSeq + 1,6), []);
                        InsertRow_TransiConfig{2,:} = {0, 0, TPOM(IntepPointSeq + 1,4), 0, [], [], EulerAngle_q11_theta(4), EulerAngle_q11_theta(4)};                        
                        NumberofInsertRows = 2;
                    case 4 % 2T2R-5-Bar
                        InsertRow_TransiConfig = {TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 5 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig = {TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 6 % 2R-SerialA1C1
                        InsertRow_TransiConfig{1,:} = {0, 0, TPOM(IntepPointSeq, 4), 0, [], [], 0, 0};
                        NumberofInsertRows = 1;
                    case 7 % 2R-SerialA2C2
                        InsertRow_TransiConfig{1,:} = {0, 0, TPOM(IntepPointSeq, 4), 0, [], [], 0, 0};
                        NumberofInsertRows = 1;                        
                    case 8 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {0, 0, 0, 0, [], []};
                        NumberofInsertRows = 1;                        
                end                 
            case 2 % 2T2R-6-Bar   
                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq,2), TPOM(IntepPointSeq,3), TPOM(IntepPointSeq,4), [], [], 0};
                        NumberofInsertRows = 1;                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 7 % 2R-SerialA2C2
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
            case 3 % 2T2R-6-Bar(xy=0)
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 7 % 2R-SerialA2C2
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 4 % 2T2R-5-Bar                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 7 % 2R-SerialA2C2
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 5 % 2T1R-3-BarSerial
                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 7 % 2R-SerialA2C2
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
                
            case 6 % 2R-SerialA1C1                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 7 % 2R-SerialA2C2
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
                
                
            case 7 % 2R-SerialA2C2
                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 8 % Fixed-SerialA1C1A2C2
                        
                end 
                
            case 8 % Fixed-SerialA1C1A2C2
                
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T1R
                        
                    case 2 % 2T2R-6-Bar
                        
                    case 3 % 2T2R-6-Bar(xy=0)
                        
                    case 4 % 2T2R-5-Bar
                        
                    case 5 % 2T1R-3-BarSerial
                        
                    case 6 % 2R-SerialA1C1
                        
                    case 7 % 2R-SerialA2C2
                        
                end 
                
                
        end
        RowSequenceoftheFirstInsertRow = IntepPointSeq + 1 + sum(InsertRowInfos(:,1));
        InsertRowInfos(IntepPointSeq,:) = [NumberofInsertRows, RowSequenceoftheFirstInsertRow];
        
        % Insert the transition configurations in the original cell: Mode_Pos_Ori_TrajPoints_cell
        MPOTP_cell = insertRow(MPOTP_cell, InsertRow_TransiConfig,NumberofInsertRows, RowSequenceoftheFirstInsertRow);
     
    end    
    
end
end

function c = insertRow(a, b, Num, row)
for i = 1 : length(a) + Num
    if i < row
        c{i, :} = a{i, :};
    elseif i == row
        for j = 1 :Num
            c{i + j - 1, :} = b{j, :};
        end
    elseif i > row + Num - 1
        c{i, :} = a{i - Num, :};
    end
end
end