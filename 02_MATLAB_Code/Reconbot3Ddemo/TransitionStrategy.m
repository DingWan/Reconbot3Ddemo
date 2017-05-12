
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
%%-------------------------------------------------------------------

%MPOTP_cell{1} = {Mode_previous,Posture_previous};
MPOTP_cell{1,:} = {Mode_current,Posture_current};
% 
Self_adjustment_Enable_Disable = 0;
NumberofInsertRows = 0;
RowSequenceoftheFirstInsertRow = 0;
InsertRowInfos = [0, 0];
   
% Here we calculate the Modes 10 and 11 in transition position {0, 30, [], [], [], []}
if Mode_previous == 10 || Mode_current == 10
    po_2RserialA1C1 = {0, -30, [], [], [], [], 0};
    obj2RserialA1C1 = RCB2RserialA1C1(po_2RserialA1C1,[],l1,l2);
    [PosOri_A1C1, ~, ~, ~, ~] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
    PosOri_2RserialA1C1_x_0_y_Positive30 = {PosOri_A1C1(1),PosOri_A1C1(2),PosOri_A1C1(3), [], [], PosOri_A1C1(6)};
elseif Mode_previous == 11 || Mode_current == 11
    po_2RserialA2C2 = {0, 30, [], [], [], [], 0};
    obj2RserialA2C2 = RCB2RserialA2C2(po_2RserialA2C2,[],l1,l2);
    [PosOri_A2C2, ~, ~, ~, ~] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
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
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{1} <= 0 
                            InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], pi/2 -pi/2}};
                        else
                            InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], -pi/2 pi/2}};
                        end              
                        MPOTP_cell{1,:} = {9, {Posture_current{1}, [], Posture_current{3}, [], Posture_current{5}, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 0 0];
                    case 10 % 2R-SerialA1C1
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0, -30, 77.4730662875815, [], [], 0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                       
                    case 11 % 2R-SerialA2C2
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0, 30, 77.4730662875815, [], [], -0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                        
                    case 12 % Fixed-SerialA1C1A2C2
                        
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
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        if MPOTP_cell{1}{2}{1} <= 0 
                            InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], pi/2 -pi/2}};
                        else
                            InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], -pi/2 pi/2}};
                        end              
                        MPOTP_cell{1,:} = {9, {Posture_current{1}, [], Posture_current{3}, [], Posture_current{5}, []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 0 0];                   
                    case 10 % 2R-SerialA1C1
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0, -30, 77.4730662875815, [], [], 0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                         
                    case 11 % 2R-SerialA2C2
                        InsertRow_TransiConfig{1,:} = {1, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{2,:} = {5, {0 0 253.3124 0 [] [], 0 0}};
                        InsertRow_TransiConfig{3,:} = {6, {0, 30, 77.4730662875815, [], [], -0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [3 1 1 1];                          
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end
            case 3 % 3T1R-SingularityA1C1
                switch Mode_current
                    case 1 % 3T2R
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1];                       
                    case 2 % 3T1R
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
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
                        InsertRow_TransiConfig{2,:} = {2, {Posture_current{1}, Posture_current{2}, Posture_current{3}, 0, [] , []}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [1 1 0];                   
                    case 7 % 2T2R-6-Bar(xy=0) 
                        InsertRow_TransiConfig{1,:} = {3, {0, 0, Posture_current{3}, 0, [] , [], 0, 0}};                      
                        InsertRow_TransiConfig{2,:} = {5, {0, 0, Posture_current{3}, 0, [] , [], 0, 0}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);
                        Self_adjustment_Enable_Disable = [0 1 1];                         
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
                        InsertRow_TransiConfig{1,:} = {4, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2]; 
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
                        %We need to intepolate on shpere surface
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0, -30, 77.4730662875815, [], [], 0.738901380334221}};                         
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);  
                        Self_adjustment_Enable_Disable = [1 1 1]; 
                    case 11 % 2R-SerialA2C2
                        %We need to intepolate on shpere surface
                        InsertRow_TransiConfig{1,:} = {5, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0, 30, 77.4730662875815, [], [], -0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);  
                        Self_adjustment_Enable_Disable = [1 1 1];                      
                    case 12 % Fixed-SerialA1C1A2C2
                        
                end   
                
            case 6 % 2T2R-6-Bar
                
                switch Mode_current
                    case 1 % 3T2R
                        
                    case 2 % 3T1R
                        
                    case 3 % 3T1R-SingularityA1C1
                        
                    case 4 % 3T1R-SingularityA2C2
                        
                    case 5 % 3T1R-SingularityA1C1A2C2 
                        InsertRow_TransiConfig{1,:} = {6, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        Self_adjustment_Enable_Disable = 0;
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
                        InsertRow_TransiConfig{1,:} = {7, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
                    case 6 % 2T2R-6-Bar
                        
                    case 7 % 2T2R-6-Bar(xy=0) 
                        Self_adjustment_Enable_Disable = 0;
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
                        InsertRow_TransiConfig{1,:} = {8, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
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
                        InsertRow_TransiConfig{1,:} = {9, Posture_previous};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig);     
                        Self_adjustment_Enable_Disable = [2 2];
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
                        InsertRow_TransiConfig{1,:} = {10, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0, -30, 77.4730662875815, [], [], 0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 2];
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
                        InsertRow_TransiConfig{1,:} = {11, Posture_previous};
                        InsertRow_TransiConfig{2,:} = {6, {0, 30, 77.4730662875815, [], [], -0.738901380334221}};
                        MPOTP_cell = exchange(MPOTP_cell, InsertRow_TransiConfig); 
                        Self_adjustment_Enable_Disable = [2 2 2];
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

function b = exchange(b,c)

d = b{1,:};
for Len_c = 1:length(c)
b{Len_c,:} = c{Len_c,:};
end
b{Len_c + 1,:} = d;

end
