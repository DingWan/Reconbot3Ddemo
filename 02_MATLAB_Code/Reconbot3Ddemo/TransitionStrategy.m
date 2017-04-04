
%%----------2-RER Transition strategy among different modes----------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 13 Dec 2016.
%     The copyright is belong to Wan Ding and IGM.

%%-------------------------------------------------------------------
% TrajPointsAndOperationalModes(NumTP,:) = [s, Pos_Ori, q0q1q2];

function  [MPOTP_cell, Modes_q0q1q2_cell] = TransitionStrategy(TPOM, Modes_q0q1q2_cell, MPOTP_cell, NumTP)

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

% Modes_q0q1q2_cell_InsertRow = Modes_q0q1q2_cell;

NumberofInsertRows = 0;
RowSequenceoftheFirstInsertRow = 0;
InsertRowInfos = [0, 0];

for IntepPointSeq = 1 : NumTP 
    
    %%-------------Step 1st: Calculate the SplinePoints: i and i+1 ----------------------
    % str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    
    
    %%-------------Step 2nd: choose spline times: 1 or 2(mode change) -------------------
    if TPOM(IntepPointSeq,1) == TPOM(IntepPointSeq+1,1) && TPOM(IntepPointSeq,1) < 3 || TPOM(IntepPointSeq,1) > 5
        continue;
    else
        switch TPOM(IntepPointSeq,1)
            case 1 % 3T2R
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T2R
                        
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq + 1,1), TPOM(IntepPointSeq + 1,2), TPOM(IntepPointSeq + 1,3), TPOM(IntepPointSeq + 1, 4), TPOM(IntepPointSeq + 1, 5), [], [], 0, []};
                        InsertRow_q0q1q2 = {0};
                        NumberofInsertRows = 1;
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
            
            case 2 % 3T1R
                switch TPOM(IntepPointSeq + 1,1)
                    
                    case 1 % 3T2R 

                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2
                        
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
                        InsertRow_TransiConfig = {TPOM(IntepPointSeq + 1,1), TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
                        NumberofInsertRows = 1;
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig = {TPOM(IntepPointSeq + 1,1), TPOM(IntepPointSeq + 1,2), 0, TPOM(IntepPointSeq + 1,4), 0, [], []};
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
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq + 1,1), TPOM(IntepPointSeq + 1,2), TPOM(IntepPointSeq + 1,3), TPOM(IntepPointSeq + 1, 4), 0, [], [], 0, 0};
                        InsertRow_q0q1q2 = {0};
                        NumberofInsertRows = 1;                         
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
                
            case 4 % 3T1R-SingularityA2C2
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T2R
                    
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2                        
                        InsertRow_TransiConfig{1,:} = {TPOM(IntepPointSeq + 1,1), TPOM(IntepPointSeq + 1,2), TPOM(IntepPointSeq + 1,3), TPOM(IntepPointSeq + 1, 4), 0, [], [], [], TPOM(IntepPointSeq + 1, 14)};
                        InsertRow_q0q1q2{1,:} = {0};
                        NumberofInsertRows = 1; 
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        
                    case 8 % 2T2R-5-Bar
                        
                    case 9 % 2T1R-3-BarSerial
                        
                    case 10 % 2R-SerialA1C1
                        
                    case 11 % 2R-SerialA2C2
                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end
                
            case 5 % 3T1R-SingularityA1C1A2C2
                switch TPOM(IntepPointSeq + 1,1)
                    case 1 % 3T2R
                        %We need to intepolate 3D space
                        
                    case 2 % 3T1R
                        %We need to intepolate 3D space
                        
                    case 3 % 3T1R-SingularityA1C1
                        %We need to intepolate on a cylinder surface    
                        
                    case 4 % 3T1R-SingularityA2C2
                         %We need to intepolate on a cylinder surface   
                         
                    case 5 % 3T1R-SingularityA1C1A2C2 
                         %We need to intepolate on a line along z-axis
                         
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
                
                switch TPOM(IntepPointSeq + 1,1)
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
                switch TPOM(IntepPointSeq + 1,1)
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
                switch TPOM(IntepPointSeq + 1,1)
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
                
                switch TPOM(IntepPointSeq + 1,1)
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
                switch TPOM(IntepPointSeq + 1,1)
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
                
                switch TPOM(IntepPointSeq + 1,1)
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
                switch TPOM(IntepPointSeq + 1,1)
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
            RowSequenceoftheFirstInsertRow = IntepPointSeq + 1 + sum(InsertRowInfos(:,1));
            InsertRowInfos(IntepPointSeq,:) = [NumberofInsertRows, RowSequenceoftheFirstInsertRow];
            
            % Insert the transition configurations in the original cell: Mode_Pos_Ori_TrajPoints_cell
            MPOTP_cell = insertRow(MPOTP_cell, InsertRow_TransiConfig,NumberofInsertRows, RowSequenceoftheFirstInsertRow);
            %Insert the output q0q1q2 value
            Modes_q0q1q2_cell = insertRow(Modes_q0q1q2_cell, InsertRow_q0q1q2, NumberofInsertRows, RowSequenceoftheFirstInsertRow);
            
        end
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