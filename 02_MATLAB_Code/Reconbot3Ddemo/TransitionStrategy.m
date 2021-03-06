
%%----------2-RER Transition strategy among different modes----------
%%-------------------------------------------------------------------

%
%     This is written by Wan Ding in 13 Dec 2016.
%     Version 1.1

%%-------------------------------------------------------------------
% TrajPointsAndOperationalModes(NumTP,:) = [s, Pos_Ori, q0q1q2];

function  [ MPOTP_cell, Self_adjustment_Enable_Disable] = TransitionStrategy(Mode_previous,Posture_previous, q0q1q2_previous, Mode_current,Posture_current, l1, l2)

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

%       1. 'Self_adjustment_Enable_Disable == 1': From HomePosition to other mode
%       2. 'Self_adjustment_Enable_Disable == 2': From other mode to HomePosition
%       3. 'Self_adjustment_Enable_Disable == 3': Random Mode to Random Mode (without the situations 1 and 2)
%       4. 'Self_adjustment_Enable_Disable == 0': No self-adjustment
%%-------------------------------------------------------------------

%MPOTP_cell{1} = {Mode_previous,Posture_previous};
MPOTP_cell{1,:} = {Mode_current,Posture_current};
% 
Self_adjustment_Enable_Disable = 0;
NumberofInsertRows = 0;
RowSequenceoftheFirstInsertRow = 0;
InsertRowInfos = [0, 0];
   
% Here we calculate the Modes 10 and 11 in transition position {0, 0.03, [], [], [], []}
if Mode_previous == 10 || Mode_current == 10
    po_2RserialA1C1 = {0, -0.03, [], [], [], [], 0};
    obj2RserialA1C1 = RCB2RserialA1C1(po_2RserialA1C1,[],l1,l2);
    [PosOri_A1C1, ~, ~, q1q2_A1C1, ~] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
    q1q2_x_0_y_Positive30 = q1q2_A1C1;
    PosOri_2RserialA1C1_x_0_y_Positive30 = {PosOri_A1C1(1),PosOri_A1C1(2),PosOri_A1C1(3), [], [], PosOri_A1C1(6)};
elseif Mode_previous == 11 || Mode_current == 11
    po_2RserialA2C2 = {0, 0.03, [], [], [], [], 0};
    obj2RserialA2C2 = RCB2RserialA2C2(po_2RserialA2C2,[],l1,l2);
    [PosOri_A2C2, ~, ~, q1q2_A2C2, ~] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
    q1q2_x_0_y_Positive30 = q1q2_A2C2;
    PosOri_2RserialA2C2_x_0_y_Negative30 = {PosOri_A2C2(1),PosOri_A2C2(2), PosOri_A2C2(3), [], [], PosOri_A2C2(6)};
end

    %%-------------Step 1st: Calculate the SplinePoints: i and i+1 ----------------------
    % str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
    
    %%-------------Step 2nd: choose spline times: 1 or 2(mode change) -------------------
        switch Mode_previous
            case 1 % 3T2R
                switch Mode_current
                    case 1 % 3T2R
                        Self_adjustment_Enable_Disable = 0;
                    case 2 % 3T1R
                        Self_adjustment_Enable_Disable = 0;
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 3];
                    case 4 % 3T1R-SingularityA2C2                        
                        InsertRow_TransiConfig{1,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 3];
                    case 5 % 3T1R-SingularityA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {1, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar     
                        Self_adjustment_Enable_Disable = 0;
                    case 7 % 2T2R-6-Bar(xy=0)
                        %
                        InsertRow_TransiConfig{1,:} = {7, {[], [], MPOTP_cell{1}{2}{3}, [], MPOTP_cell{1}{2}{5}, MPOTP_cell{1}{2}{6} }};
                        %
                        MPOTP_cell{1,:} = {6, {0, 0, MPOTP_cell{1}{2}{3}, [], [], MPOTP_cell{1}{2}{6}}};
                        MPOTP_cell{2,:} = InsertRow_TransiConfig{1,:};
                        %
                        Self_adjustment_Enable_Disable = [0 0];
                    case 8 % 2T2R-5-Bar
                        %Here we use: p_BinaryCode = [1 1 1 0 1 0] ( [p(1), p(2), p(3), [], beta, []])
                        MPOTP_cell{1,:} =  {6, {MPOTP_cell{1}{2}{1}, MPOTP_cell{1}{2}{2}, MPOTP_cell{1}{2}{3}, [], MPOTP_cell{1}{2}{5}, []}};
                        Self_adjustment_Enable_Disable = 0;
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{1} <= 0 
                            InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], pi/2 -pi/2}};
                        else
                            InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], -pi/2 pi/2}};
                        end              
                        MPOTP_cell{1,:} = {9, {Posture_current{1}, [], Posture_current{3}, [], Posture_current{5}, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 0 0];
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                       
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                        
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {1, {0 0 Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end
                end
            
            case 2 % 3T1R
                switch Mode_current                    
                    case 1 % 3T2R 
                        Self_adjustment_Enable_Disable = 0;
                    case 2 % 3T1R
                        Self_adjustment_Enable_Disable = 0;
                    case 3 % 3T1R-SingularityA1C1                        
                        InsertRow_TransiConfig{1,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 3];
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 3];                      
                    case 5 % 3T1R-SingularityA1C1A2C2                        
                        InsertRow_TransiConfig{1,:} = {2, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {2, {Posture_current{1}, Posture_current{2}, Posture_current{3}, 0, [], []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                     
                    case 7 % 2T2R-6-Bar(xy=0)
                        %
                        InsertRow_TransiConfig{1,:} = {2, {Posture_previous{1}, Posture_previous{2}, Posture_previous{3}, 0, [], []}};
                        InsertRow_TransiConfig{2,:} = {7, {[], [], MPOTP_cell{1}{2}{3}, [], MPOTP_cell{1}{2}{5}, MPOTP_cell{1}{2}{6}}};
                        %
                        MPOTP_cell{2,:} = {6, {0, 0, MPOTP_cell{1}{2}{3}, [], [], MPOTP_cell{1}{2}{6}}};
                        MPOTP_cell{3,:} = InsertRow_TransiConfig{2,:};
                        MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                        %
                        Self_adjustment_Enable_Disable = [0 0 0];                        
                    case 8 % 2T2R-5-Bar                        
                        InsertRow_TransiConfig{1,:} = {2, {Posture_previous{1}, Posture_previous{2}, Posture_previous{3}, 0, [], []}};
                        %Here we use: p_BinaryCode = [1 1 1 0 1 0] ( [p(1), p(2), p(3), [], beta, []])
                        MPOTP_cell{2,:} =  {6, {MPOTP_cell{1}{2}{1}, MPOTP_cell{1}{2}{2}, MPOTP_cell{1}{2}{3}, [], MPOTP_cell{1}{2}{5}, []}};
                        MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                        Self_adjustment_Enable_Disable = [0 0];                        
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{1} <= 0 
                            InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], pi/2 -pi/2}};
                        else
                            InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], -pi/2 pi/2}};
                        end              
                        MPOTP_cell{1,:} = {9, {Posture_current{1}, [], Posture_current{3}, [], Posture_current{5}, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 0 0];                   
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                         
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {1, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 208.879343162506 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, q21, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                          
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {2, {0 0 Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end
                end
            case 3 % 3T1R-SingularityA1C1
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {3, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];                       
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {3, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];
                    case 3 % 3T1R-SingularityA1C1
                        Self_adjustment_Enable_Disable = 0;
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {3, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1 3];                        
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {3, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2]; 
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {3, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {2, {Posture_current{1}, Posture_current{2}, Posture_current{3}, 0, [], []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1 0];                   
                    case 7 % 2T2R-6-Bar(xy=0) 
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};                      
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 8 % 2T2R-5-Bar
                        if q0q1q2_previous(7) > 0
                            q11 = pi/2;
                            q21 = pi/2;
                        else
                            q11 = -pi/2;
                            q21 = -pi/2;
                        end
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_current{3}, 0, [], [], q11, q21}}; 
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];
                    case 9 % 2T1R-3-BarSerial
                        %Here, the value of q11 and q12 is decided by the x value of current target position
                        if q0q1q2_previous(7) > 0
                            q11 = pi/2;
                            q21 = pi/2; 
                            if Posture_current{1} < 0
                                q12_Adjust = -pi/2;
                            else
                                q12_Adjust = pi/2;
                            end
                        else
                            q11 = -pi/2;
                            q21 = -pi/2; 
                            if Posture_current{1} < 0
                                q12_Adjust = -pi/2;
                            else
                                q12_Adjust = pi/2;
                            end
                        end
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_current{3}, 0, [], [], q11, q21}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], q11, q12_Adjust}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0 0];
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_previous{3}, 0, [], [], q11, 0}}; 
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], q11 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_previous{3}, 0, [], [], 0, q21}}; 
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], 0, q21}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {3, {0 0 Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end
                end  
                
            case 4 % 3T1R-SingularityA2C2
                switch Mode_current
                    case 1 % 3T2R                    
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];  
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];                        
                    case 3 % 3T1R-SingularityA1C1                        
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {2, Posture_current};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1 3]; 
                    case 4 % 3T1R-SingularityA2C2 
                        Self_adjustment_Enable_Disable = 0;
                    case 5 % 3T1R-SingularityA1C1A2C2  
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2]; 
                    case 6 % 2T2R-6-Bar                        
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {2, {Posture_current{1}, Posture_current{2}, Posture_current{3}, 0, [], []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1 0];   
                    case 7 % 2T2R-6-Bar(xy=0)                         
                        InsertRow_TransiConfig{1,:} = {4, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};                      
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];   
                    case 8 % 2T2R-5-Bar
                        if q0q1q2_previous(2) > 0
                            q11 = pi/2;
                            q21 = pi/2;
                        else
                            q11 = -pi/2;
                            q21 = -pi/2;
                        end
                        InsertRow_TransiConfig{1,:} = {4, {0, 0, Posture_current{3}, 0, [], [], q11, q21}}; 
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                        
                    case 9 % 2T1R-3-BarSerial                        
                        %Here, the value of q11 and q12 is decided by the x value of current target position
                        if q0q1q2_previous(2) > 0
                            q11 = pi/2;
                            q21 = pi/2; 
                            if Posture_current{1} < 0
                                q11_Adjust = pi/2;
                            else
                                q11_Adjust = -pi/2;
                            end
                        else
                            q11 = -pi/2;
                            q21 = -pi/2; 
                            if Posture_current{1} < 0
                                q11_Adjust = pi/2;
                            else
                                q11_Adjust = -pi/2;
                            end
                        end
                        InsertRow_TransiConfig{1,:} = {4, {0, 0, Posture_current{3}, 0, [], [], q11, q21}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], q11_Adjust, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0 0];
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {4, {0, 0, Posture_previous{3}, 0, [], [], q11 0}}; 
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], q11 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                        
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {4, {0, 0, Posture_previous{3}, 0, [], [], 0, q21}}; 
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], 0, q21}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                        
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {4, {0 0 Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end                        
                end
                
            case 5 % 3T1R-SingularityA1C1A2C2
                switch Mode_current
                    case 1 % 3T2R
                        %We need to intepolate 3D space
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};   
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];                        
                    case 2 % 3T1R
                        %We need to intepolate 3D space
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];                        
                    case 3 % 3T1R-SingularityA1C1
                        %We need to intepolate on a cylinder surface 
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1]; 
                    case 4 % 3T1R-SingularityA2C2
                         %We need to intepolate on a cylinder surface 
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];  
                    case 5 % 3T1R-SingularityA1C1A2C2 
                         %We need to intepolate on a line along z-axis
                         Self_adjustment_Enable_Disable = 0;
                    case 6 % 2T2R-6-Bar
                        %We need to intepolate 3D space  
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];
                    case 7 % 2T2R-6-Bar(xy=0)                         
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];
                    case 8 % 2T2R-5-Bar
                         %We need to intepolate on o-xz plane
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];
                    case 9 % 2T1R-3-BarSerial
                        %We need to intepolate on o-xz plane
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [1 1];
                    case 10 % 2R-SerialA1C1                        
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        %We need to intepolate on shpere surface
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};                         
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);  
                        Self_adjustment_Enable_Disable = [1 1 1]; 
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        %We need to intepolate on shpere surface
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);  
                        Self_adjustment_Enable_Disable = [1 1 1];                      
                    case 12 % Fixed-SerialA1C1A2C2
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                           InsertRow_TransiConfig{1,:} = {5, {0 0 0 0 [] [] 0 0}}; 
                           MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                           Self_adjustment_Enable_Disable = [0 0];
                        else
                            MPOTP_cell{1}{1} = 5;
                            MPOTP_cell{1}{2}{4} = 0;
                            Self_adjustment_Enable_Disable = 0;
                        end                        
                end   
                
            case 6 % 2T2R-6-Bar                
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {6, {Posture_previous{1:3}, [], [], 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                        
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {6, {Posture_previous{1:3}, [], [], 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                          
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], Posture_current{7} 0}};
                        if MPOTP_cell{1}{2}{1} > 0
                            MPOTP_cell{1}{2}{8} = pi/2;
                        else
                            MPOTP_cell{1}{2}{8} = -pi/2;
                        end
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                          
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 Posture_previous{3} 0 [] [], 0 Posture_current{8}}};
                        if MPOTP_cell{1}{2}{1} > 0
                            MPOTP_cell{1}{2}{7} = pi/2;
                        else
                            MPOTP_cell{1}{2}{7} = -pi/2;
                        end
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {6, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        Self_adjustment_Enable_Disable = 0;
                    case 7 % 2T2R-6-Bar(xy=0) 
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], Posture_previous{6}}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0]; 
                    case 8 % 2T2R-5-Bar
                        if q0q1q2_previous(2) ~= q0q1q2_previous(7)
                            InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                            if q0q1q2_previous(7) > 0
                                InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], [], pi/2, pi/2}};
                            else                                
                                InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], [], -pi/2, -pi/2}};
                            end
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [3 0 0];
                        else
                            InsertRow_TransiConfig{1,:} = {6, {Posture_current{1}, 0, Posture_current{3}, [], [], Posture_current{6}}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];
                        end
                    case 9 % 2T1R-3-BarSerial
                        if q0q1q2_previous(2) ~= q0q1q2_previous(7)
                            InsertRow_TransiConfig{1,:} = {6, {Posture_current{1}, 0, Posture_current{3}, [], [], 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];                            
                        else                            
                            InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                            if q0q1q2_previous(7) > 0
                                InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], [], pi/2, pi/2}};
                            else                                
                                InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], [], -pi/2, -pi/2}};
                            end
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [3 0 0];
                        end                       
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], []}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1]; 
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3}, 0, [], []}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                         
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {6, {0, 0, Posture_previous{3}, [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end                          
                end 
                
            case 7 % 2T2R-6-Bar(xy=0)
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                     
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 3 % 3T1R-SingularityA1C1                        
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        %MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];  
                    case 4 % 3T1R-SingularityA2C2                        
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{7} = 0;
                        %MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                        
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {7, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar                        
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1]; 
                    case 7 % 2T2R-6-Bar(xy=0) 
                        Self_adjustment_Enable_Disable = 0;
                    case 8 % 2T2R-5-Bar
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                        
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                         
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                         
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {7, {[], [], Posture_previous{3}, [], 0, 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end                         
                end                 
                
            case 8 % 2T2R-5-Bar                
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {8, {Posture_previous{1:3}, [], 0, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];  
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {8, {Posture_previous{1:3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {1, {Posture_current{1:3}, 0, [], []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0 0];  
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        InsertRow_TransiConfig{1,:} = {8, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {8, {Posture_previous{1:3}, [], 0, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                          
                    case 8 % 2T2R-5-Bar
                        Self_adjustment_Enable_Disable = 0;
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 10 % 2R-SerialA1C1                        
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];  
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                         
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {8, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end                         
                end                 
                
            case 9 % 2T1R-3-BarSerial                
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {9, {Posture_previous{1:3}, [], 0, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                     
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {9, {Posture_previous{1:3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {1, {Posture_current{1:3}, 0, [], []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0 0];                         
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};                        
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {9, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {9, {Posture_previous{1:3}, [], 0, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 0];                         
                    case 7 % 2T2R-6-Bar(xy=0)                         
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}}; 
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];
                    case 8 % 2T2R-5-Bar
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
                    case 9 % 2T1R-3-BarSerial
                        Self_adjustment_Enable_Disable = 0;
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) > 0
                            q11 = pi;
                        else
                            q11 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                          
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(7) > 0
                            q21 = pi;
                        else
                            q21 = -pi;
                        end
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_previous{3} 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};  
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1 1];                         
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {9, {0, 0, Posture_previous{3}, [], 0, []}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 0 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end                              
                end 
                
            case 10 % 2R-SerialA1C1 
                if q0q1q2_previous(2) > 0
                    q11 = pi;
                else
                    q11 = -pi;
                end
                switch Mode_current                    
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                    
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell{1}{2}{7} = -pi;
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        %MPOTP_cell{1}{2}{7} = -pi;
                        MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell{1}{2}{7} = -pi;
                        %MPOTP_cell{1}{2}{8} = 0;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                         
                    case 5 % 3T1R-SingularityA1C1A2C2    
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 2];
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                        
                    case 7 % 2T2R-6-Bar(xy=0)
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                         
                    case 8 % 2T2R-5-Bar
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                         
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,-30.0000000000000,77.4865794831595,[],[],0.738783927444496, q11, 0}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, q11, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                        
                    case 10 % 2R-SerialA1C1
                        Self_adjustment_Enable_Disable = 0;
                    case 11 % 2R-SerialA2C2
                        InsertRow_TransiConfig{1,:} = {10, {0, 0, [], [], [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {12, {0, 0, 0, [] [] [], 0, 0, -pi, 0}};
                        InsertRow_TransiConfig{3,:} = {11, {0,30.0000000000000,[],[],[],[], 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [0 0 0 0];                         
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {10, {0, 0, [], [], [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {12, {0, 0, 0, [] [] [], 0, 0, 0, 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end 
                end 
                
                
            case 11 % 2R-SerialA2C2
                if q0q1q2_previous(2) > 0
                    q21 = pi;
                else
                    q21 = -pi;
                end                
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                      
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell{1}{2}{8} = -pi;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 3 % 3T1R-SingularityA1C1
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        %MPOTP_cell{1}{2}{7} = 0;
                        MPOTP_cell{1}{2}{8} = -pi;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 4 % 3T1R-SingularityA2C2
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell{1}{2}{7} = 0;
                        %MPOTP_cell{1}{2}{8} = -pi;
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                         
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 2];
                    case 6 % 2T2R-6-Bar
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 7 % 2T2R-6-Bar(xy=0)
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 0];                          
                    case 8 % 2T2R-5-Bar
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 9 % 2T1R-3-BarSerial
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0,30.0000000000000,77.4865794831595,[],[],-0.738783927444496, 0, q21}};
                        InsertRow_TransiConfig{3,:} = {5, {0, 0, 208.879343162506 0 [] 0, 0, q21}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 1 1];                          
                    case 10 % 2R-SerialA1C1
                        InsertRow_TransiConfig{1,:} = {11, {0, 0, [], [], [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {12, {0, 0, 0, [] [] [], -pi, 0, 0, 0}};
                        InsertRow_TransiConfig{3,:} = {10, {0,-30.0000000000000,[],[],[],[], 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [0 0 0 0];                         
                    case 11 % 2R-SerialA2C2
                        Self_adjustment_Enable_Disable = 0;
                    case 12 % Fixed-SerialA1C1A2C2
                        InsertRow_TransiConfig{1,:} = {11, {0, 0, [], [], [], [], 0}};
                        InsertRow_TransiConfig{2,:} = {12, {0, 0, 0, [] [] [], 0, 0, 0, 0}};
                        if MPOTP_cell{1}{2}{7} ~= 0 || MPOTP_cell{1}{2}{8} ~= 0 || MPOTP_cell{1}{2}{9} ~= 0 || MPOTP_cell{1}{2}{10} ~= 0
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            MPOTP_cell{1,:} = InsertRow_TransiConfig{1,:};
                            MPOTP_cell{2,:} = InsertRow_TransiConfig{2,:};
                            Self_adjustment_Enable_Disable = [0 0];
                        end 
                end 
                
            case 12 % Fixed-SerialA1C1A2C2                
                switch Mode_current
                    case 1 % 3T2R
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end
                    case 2 % 3T1R
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                        
                    case 3 % 3T1R-SingularityA1C1
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                              
                    case 4 % 3T1R-SingularityA2C2
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                          
                    case 5 % 3T1R-SingularityA1C1A2C2
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];
                        else                            
                            Self_adjustment_Enable_Disable = 0;
                        end
                    case 6 % 2T2R-6-Bar
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                          
                    case 7 % 2T2R-6-Bar(xy=0)
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];
                        end 
                    case 8 % 2T2R-5-Bar
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                            
                    case 9 % 2T1R-3-BarSerial
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}}; 
                            InsertRow_TransiConfig{3,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 1 1];
                        else                            
                            InsertRow_TransiConfig{1,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [], [], 0, 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 1 1];
                        end                            
                    case 10 % 2R-SerialA1C1
                        if q0q1q2_previous(2) ~= -pi || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= 0 || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], -pi, 0, 0, 0}};
                            InsertRow_TransiConfig{2,:} = {10, {0,-30.0000000000000,[],[],[],[], 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            InsertRow_TransiConfig{1,:} = {10, {0,-30.0000000000000,[], [], [], [], 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];
                        end                            
                    case 11 % 2R-SerialA2C2
                        if q0q1q2_previous(2) ~= 0 || q0q1q2_previous(3) ~= 0 || q0q1q2_previous(7) ~= -pi || q0q1q2_previous(8) ~= 0
                            InsertRow_TransiConfig{1,:} = {12, {0, 0, 0 [] [] [], 0, 0, -pi, 0}};
                            InsertRow_TransiConfig{2,:} = {11, {0,30.0000000000000,[],[],[],[], 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0 0];
                        else
                            InsertRow_TransiConfig{1,:} = {11, {0,30.0000000000000,[], [], [], [], 0}};
                            MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                            Self_adjustment_Enable_Disable = [0 0];
                        end                          
                    case 12 % Fixed-SerialA1C1A2C2
                          Self_adjustment_Enable_Disable = 0;                    
                end 
                
        end
        if NumberofInsertRows ~= 0
            
            for i = 1:NumberofInsertRows                
                % Insert the transition configurations in the original cell: Mode_Pos_Ori_TrajPoints_cell
                MPOTP_cell{i+1,:} = InsertRow_TransiConfig{i,:};
            end
            
        end   
end

function b = exchange(b,c)

d = b{1,:};
for Len_c = 1:length(c)
    b{Len_c,:} = c{Len_c,:};
end
b{Len_c + 1,:} = d;

end
