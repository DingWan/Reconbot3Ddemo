function [ q0q1q2_P2P ] = MotionPlanningOptimalSoultion_debugging(Mode_previous, PosOri_previous, q0q1q2_previous_trajpoint,...
                                                        Mode_current,PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Time, l1, l2)
% Motion planning and Optimal Solution
%
%    Mode_previous: The mode (1~12) of previous one step;
% Posture_previous: The Posture (Position + Orientation + q11 + q21) of previous one step;
%  q0q1q2_previous: The joints value [q0, q11~q15, q21~q25] of previous one step;
%     Mode_current: The mode (1~12) of previous one step;
%  Posture_current: The Posture (Position + Orientation + q11 + q21) of previous one step;
%   q0q1q2_current: The joints value [q0, q11~q15, q21~q25] of previous one step;
%   NumIntepoPoint: Current intepotation number
%             Time: Intepotation Time

%% Transition Strategy
%[ MPOTP_cell, Self_adjustment_Enable_Disable ] = TransitionStrategy(Mode_previous,PosOri_previous, q0q1q2_previous_trajpoint, Mode_current,PosOri_current);
[ MPOTP_cell, Self_adjustment_Enable_Disable_Array ] = TransitionStrategy_debugging(Mode_previous,PosOri_previous, q0q1q2_previous_trajpoint, Mode_current,PosOri_current);


%%
po_Intep = [];
q0q1q2_P2P = [];
Mode_previous_initial = Mode_previous;
PosOri_previous_initial = PosOri_previous;
Mode_current_initial = Mode_current;
PosOri_current_initial = PosOri_current;

for i = 1:length(MPOTP_cell)
    %% Assign value to "Self_adjustment_Enable_Disable == 0/1/2/3"
    Self_adjustment_Enable_Disable = Self_adjustment_Enable_Disable_Array(i); 
        
    %% Assgin "Mode_Previous, Mode_Current, PosOri_Current, PosOri_Previous"
    for OnlyUsedforFoldingThisPart_Mode5andMode10and11 = 1:1
        if i == 1
            if  Self_adjustment_Enable_Disable == 1
                Mode_previous = MPOTP_cell{1}{1};
                PosOri_previous = MPOTP_cell{1}{2};
                Mode_current = MPOTP_cell{2}{1};
                PosOri_current = MPOTP_cell{2}{2};
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 2
                Mode_previous = MPOTP_cell{1}{1};
                PosOri_previous = MPOTP_cell{1}{2};
                Mode_current = MPOTP_cell{2}{1};
                PosOri_current = MPOTP_cell{2}{2};
                Mode = Mode_previous;
            elseif  Self_adjustment_Enable_Disable == 3
                
            elseif  Self_adjustment_Enable_Disable == 0
                Mode = Mode_current;
            end            
        elseif i > 1
            if  Self_adjustment_Enable_Disable == 1
                Mode_previous;
                PosOri_previous;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = PosOri_self_adjustment_1;
                Mode = Mode_previous;
            elseif  Self_adjustment_Enable_Disable == 2
                Mode_previous;
                PosOri_previous = PosOri_self_adjustment_2;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};
                Mode = Mode_current;
            elseif  Self_adjustment_Enable_Disable == 3
                
            elseif  Self_adjustment_Enable_Disable == 0
                Mode_previous = Mode_current;
                PosOri_previous = PosOri_current;
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2}; 
                Mode = Mode_current;
            end
        end       
        
        
%         if Self_adjustment_Enable_Disable == 0 && Self_adjustment_Enable_Disable == 3
%             if Mode_current_initial == 7 && i == length(MPOTP_cell) % Mode1/2 to 7
%                 PosOri_previous{5} = p(5);
%             end
%         end
%         if Self_adjustment_Enable_Disable == 1 && Mode_current_initial == 4 && i == length(MPOTP_cell) && Mode_previous ~= 5 % Mode3/4 to 4/3
%             PosOri_current = MPOTP_cell{i}{2};
%             PosOri_previous = MPOTP_cell{i}{2};
%             q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(i-2),:);
%             PosOri_previous{8} = q0q1q2_OptimalRow(NumIntepoPoints*(i-2),7);
%         end
%         
%         if ( Self_adjustment_Enable_Disable == 1 && (Mode_previous == 3 || Mode_previous == 4 || Mode_previous == 5 ) ) ||...
%                 ( i == length(MPOTP_cell) && (Mode_current == 3 || Mode_current == 4 || Mode_current == 5) ) || ...
%                 Self_adjustment_Enable_Disable == 3 || Self_adjustment_Enable_Disable == 0
%             Mode = Mode_current;
%         else
%             Mode = Mode_previous;
%         end
    end
    
    %% Deal with Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi) and  Mode 10/11(2RSerialAiCi) to Mode 5(HomePosition)
    for OnlyUsedforFoldingThisPart_Mode5andMode10and11 = 1:1
        % if random mode to Mode 10/11, so,
        % Firstly, it must recovery to Mode 5(HomePosition);
        % And then, execute Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi)
        
        % Mode 5(HomePosition) to Mode 10/11(2RSerialAiCi)
        if Self_adjustment_Enable_Disable == 1 && (Mode_current_initial == 10 || Mode_current_initial == 11)
            Length_Mode10or11 = length(MPOTP_cell);
            if i == Length_Mode10or11 - 2
                Mode_previous = MPOTP_cell{i}{1};
                PosOri_previous = MPOTP_cell{i}{2};
                Mode_current = MPOTP_cell{i+1}{1};
                PosOri_current = MPOTP_cell{i+1}{2};
            elseif i == Length_Mode10or11
                Mode_previous = MPOTP_cell{i}{1};
                if MPOTP_cell{i}{1} == 10
                    if Length_Mode10or11 == 3
                        PosOri_previous = {MPOTP_cell{Length_Mode10or11 - 1}{2}{1}, MPOTP_cell{Length_Mode10or11 - 1}{2}{2}, [], [], [] ,[], q0q1q2_OptimalRow(NumIntepoPoints*(i-2),3)};
                    else
                        PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),3);
                    end
                elseif MPOTP_cell{i}{1} == 11
                    if Length_Mode10or11 == 3
                        PosOri_previous = {MPOTP_cell{Length_Mode10or11 - 1}{2}{1}, MPOTP_cell{Length_Mode10or11 - 1}{2}{2}, [], [], [] ,[], q0q1q2_OptimalRow(NumIntepoPoints*(i-2),8)};
                    else
                        if i ~= 1
                            PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),8);
                        end
                    end
                end
                Mode_current = MPOTP_cell{i}{1};
                PosOri_current = MPOTP_cell{i}{2};                
                for OnlyUsedforFoldingThisPart_Mode5toMode10and11 = 1:1
                    % ----------------------------------------------------------------------
                    % ---------- Adjust the q12/q22 from mode 5 to mode 10/11---------------
                    if  MPOTP_cell{i}{1} == 10 && Length_Mode10or11 > 1
                        % Keep the last two angles  q12 and q14  equals to the last second angles  q12 and q14
                        %--- Here we assign the value by extrapolation ----
                        % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
                        % Here we get the new "q0q1q2_OptimalRow(LastRowValue,7:11)" value by Extrapolation due to the mode switch (Singularity)
                        if Length_Mode10or11 == 3
                            Lenth = NumIntepoPoints;
                        else
                            Lenth = length(q0q1q2_OptimalRow);
                        end
                        delta_q0q1q2 = ((q0q1q2_OptimalRow(Lenth-2,:)-q0q1q2_OptimalRow(Lenth-1,:)) + (q0q1q2_OptimalRow(Lenth-3,:)-q0q1q2_OptimalRow(Lenth-2,:)))/2;
                        q0q1q2_OptimalRow(Lenth,3) = q0q1q2_OptimalRow(Lenth-1,3) - delta_q0q1q2(3);
                        PosOri_previous{7} = q0q1q2_OptimalRow(Lenth,3);
                        po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
                        obj2RserialA1C1 = RCB2RserialA1C1(po,[],l1,l2);
                        [~, ~, ~, q1q2, ~] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
                        for jj = 1:length(q1q2(:,1))
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(Lenth-1,2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(Lenth-1,7:11));
                        end
                        [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
                        [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
                        q0q1q2_OptimalRow(Lenth,:) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
                    elseif MPOTP_cell{i}{1} == 11 && Length_Mode10or11 > 1
                        % Keep the last two angles  q22 and q24  equals to the last second angles  q22 and q24
                        %--- Here we assign the value by extrapolation ----
                        % Delta = ((last third value - last Second Value) + (last Forth value - last third Value))/2
                        % Here we get the new "q0q1q2_OptimalRow(LastRowValue,7:11)" value by Extrapolation due to the mode switch (Singularity)
                        if Length_Mode10or11 == 3
                            Lenth = NumIntepoPoints;
                        else
                            Lenth = length(q0q1q2_OptimalRow);
                        end
                        delta_q0q1q2 = ((q0q1q2_OptimalRow(Lenth-2,:)-q0q1q2_OptimalRow(Lenth-1,:)) + (q0q1q2_OptimalRow(Lenth-3,:)-q0q1q2_OptimalRow(Lenth-2,:)))/2;
                        q0q1q2_OptimalRow(Lenth,8) = q0q1q2_OptimalRow(Lenth-1,8) - delta_q0q1q2(8);
                        PosOri_previous{7} = q0q1q2_OptimalRow(Lenth,8);
                        po = {PosOri_previous{1}, PosOri_previous{2}, [], [], [], [], PosOri_previous{7}};
                        obj2RserialA2C2 = RCB2RserialA2C2(po,[],l1,l2);
                        [~, ~, ~, q1q2, ~] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
                        for jj = 1:length(q1q2(:,1))
                            q1q2A1C1_norm(jj) = norm(q1q2(jj,1:5) - q0q1q2_OptimalRow(Lenth-1,2:6));
                            q1q2A2C2_norm(jj) = norm(q1q2(jj,6:10) - q0q1q2_OptimalRow(Lenth-1,7:11));
                        end
                        [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
                        [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
                        q0q1q2_OptimalRow(Lenth,1:11) = [q0, q1q2(colsA1C1(1),1:5), q1q2(colsA2C2(1),6:10)];
                    end
                    % -------------------------------------------------------------------
                end
                
                % Assign the correct value for the last self-adjustment to
                % right position
                Mode = Mode_current;
                q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(i-2),:);
                %
            end            
        end
            
        % Mode 10/11(2RSerialAiCi) to Mode 5(HomePosition)
        if Self_adjustment_Enable_Disable == 2 && (Mode_previous_initial == 10 || Mode_previous_initial == 11)
            if i == 1
                % Here we deliver two values on each step end of q0q1q2_mat to q0q1q2_previous_trajpoint
                q0q1q2_previous_i_1 = q0q1q2_previous_trajpoint(1,:);
                q0q1q2_previous_i_2 = q0q1q2_previous_trajpoint(2,:);
                q0q1q2_previous_trajpoint = q0q1q2_previous_i_2;
                Mode_previous = MPOTP_cell{1}{1};
                PosOri_previous = MPOTP_cell{1}{2};
                Mode_current = MPOTP_cell{1}{1};
                if MPOTP_cell{1}{1} == 10
                    PosOri_current = {MPOTP_cell{2}{2}{1}, MPOTP_cell{2}{2}{2}, [], [], [], [], q0q1q2_previous_i_1(3)};
                elseif MPOTP_cell{1}{1} == 11
                    PosOri_current = {MPOTP_cell{2}{2}{1}, MPOTP_cell{2}{2}{2}, [], [], [], [], q0q1q2_previous_i_1(8)};
                end
            elseif i == 2
                Mode_previous = MPOTP_cell{2}{1};
                PosOri_previous = MPOTP_cell{2}{2};
                Mode_current = MPOTP_cell{3}{1};
                PosOri_current = MPOTP_cell{3}{2};
                q0q1q2_previous_trajpoint = q0q1q2_previous_i_1;
            end
        end
         
    end
            
    %% Intepotation and Trajectory Planning
    for OnlyUsedforFoldingThisPart_TrajectoryPlanning = 1:1
       
        if ( Self_adjustment_Enable_Disable == 2 && i == length(MPOTP_cell) ) && (Mode_current == 3 || Mode_current == 4 || Mode_current == 5)
            % Mode 3/4/5 needs to adjust
            q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
        end
        
        [ po_Intep ] =  IntepotationP2P(Mode, PosOri_previous,q0q1q2_previous_trajpoint, PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, l1, l2);
        
    end
    
    %% Iterative IK solution of all intepolation points
    for j = 1:NumIntepoPoints
        %
        if Mode == Mode_current
            PosOri_Effect = PosOri_current;
        elseif Mode == Mode_previous
            PosOri_Effect = PosOri_previous;
        end
        PosOri = {};
        for k = 1:length(PosOri_Effect)
            if isempty(PosOri_Effect{k}) ~= 1
                PosOri{k} = po_Intep(k,j);
            else
                PosOri{k} = [];
            end
        end

        % We assign the q11 and q21 with previous one step value
        if i == 1
            if j == 1
                % only used for the first step from HomePosition to other mode
                q0 = q0q1q2_previous_trajpoint(1);
                q11 = q0q1q2_previous_trajpoint(2);
                q21 = q0q1q2_previous_trajpoint(7);
                q0q1q2_OptimalRow(1,:) = q0q1q2_previous_trajpoint;
            else
                %if self-adjustment exists
                q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
                if Mode == 3
                    q11 = PosOri{7};
                    q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                elseif Mode == 4
                    q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                    q21 = PosOri{8};
                elseif Mode == 5
                    if Mode_previous == 5 && Mode_previous == Mode_current
                        q11 = PosOri{7};
                        q21 = PosOri{8};
                    else
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    end
                else
                    q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                    q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                end
            end
        else
            q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
            if j == 1
                if Self_adjustment_Enable_Disable == 1
                    q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_previous_trajpoint;
                elseif Self_adjustment_Enable_Disable == 2
                    q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
                    q0q1q2_Optimal_SingleRow = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:);
                elseif Self_adjustment_Enable_Disable == 3 % 2RserialAiCi
                    q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
                    q0q1q2_previous_trajpoint = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
                end
            end
            if length(PosOri_Effect) == 7 && Mode == 10
                q11 = PosOri{7};
                q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
            elseif length(PosOri_Effect) == 7 && Mode == 11
                q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                q21 = PosOri{7};
            elseif length(PosOri_Effect) == 6
                q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
            else
                q11 = PosOri{7};
                q21 = PosOri{8};
            end
        end

        % IK solution
        [ p, ~, ~, q1q2, ~ ] = IK(Mode, PosOri, q0, q11, q21, q0q1q2_OptimalRow(length(q0q1q2_OptimalRow(:,1)),:), l1, l2);
        q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];

        % Optimal Joints Solution
        OptimalJointsSolution;
        q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j,:) = q0q1q2_Optimal_SingleRow;
        %q0q1q2_Optimal_SingleRow * 180 / pi
        %------ Show debugging
        %ReconbotANI(q0q1q2_Optimal_SingleRow);

        %------ Show Center point of Moving Platform
        %Displacement = [250,250,165.88];
        %p_Base = p(1:3) + Displacement;
        %plot3(p_Base(1),p_Base(2),p_Base(3),'r.');
    end    
    
    %% Self_adjustment mode contain three types:
    for OnlyUsedforFoldingThisPart_Self_adjustment = 1:1
        % 1. 'Self_adjustment_Enable_Disable == 1': From HomePosition to other mode
        % 2. 'Self_adjustment_Enable_Disable == 2': From other mode to HomePosition
        % 3. 'Self_adjustment_Enable_Disable == 3': Random Mode to Random Mode (without the situations 1 and 2)
        % 4. 'Self_adjustment_Enable_Disable == 0': No self-adjustment
        if  Self_adjustment_Enable_Disable == 1             
            % Here is used for executing 'Self_adjustment' only once, length(MPOTP_cell) > 3 means Modes10/11;
            if Mode == 3
                PosOri_current{7} = PosOri_previous{7};
                PosOri_current{8} = q0q1q2_OptimalRow(2,7);
            elseif Mode == 4
                PosOri_current{7} = q0q1q2_OptimalRow(2,2);
                PosOri_current{8} = PosOri_previous{8};
            else
                PosOri_current{7} = q0q1q2_OptimalRow(2,2);
                PosOri_current{8} = q0q1q2_OptimalRow(2,7);
            end
            PosOri_self_adjustment_1 = {PosOri_previous{1:6},PosOri_current{7:8}};
            if i == 1 || ( length(MPOTP_cell) > 3 && i == Length_Mode10or11 - 2)
                q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+2,:);
            end
        elseif Self_adjustment_Enable_Disable == 2 %&& ( (i == 1 && (Mode ~= 10 && Mode ~= 11)) || (i == 2 && Mode == 6) )            
            if Mode == 3
                q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
                q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            elseif Mode == 4
                q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
            elseif Mode == 6
            else
                q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
                q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
                q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            end
            PosOri_current{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
            PosOri_current{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
            PosOri_self_adjustment_2 = PosOri_current;
        elseif Self_adjustment_Enable_Disable == 0 || Self_adjustment_Enable_Disable == 3
            PosOri_previous = PosOri_current;
            if Mode_current == 3 && Mode_previous == 3
                q0q1q2_OptimalRow(NumIntepoPoints*i,2) = q0q1q2_OptimalRow(NumIntepoPoints*i - 1,2);
            elseif Mode_current == 4 && Mode_previous == 4
                q0q1q2_OptimalRow(NumIntepoPoints*i,7) = q0q1q2_OptimalRow(NumIntepoPoints*i - 1,7);
            else
                
            end
            PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
            PosOri_previous{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
        end
    end
    
end

if Self_adjustment_Enable_Disable == 1 %&& (Mode_current_initial ~= 10 && Mode_current_initial ~= 11)
    q0q1q2_P2P = q0q1q2_OptimalRow;
    q0q1q2_P2P(1:NumIntepoPoints,:) = q0q1q2_OptimalRow((NumIntepoPoints+1):2*NumIntepoPoints,:);
    q0q1q2_P2P((NumIntepoPoints+1):2*NumIntepoPoints,:) = q0q1q2_OptimalRow(1:NumIntepoPoints,:);
elseif Self_adjustment_Enable_Disable == 2
    q0q1q2_P2P = q0q1q2_OptimalRow;
else
    q0q1q2_P2P = q0q1q2_OptimalRow(1:NumIntepoPoints*i,:);
end

end