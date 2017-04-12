function [ q0q1q2_P2P ] = MotionPlanningOptimalSoultion(Mode_previous, PosOri_previous, q0q1q2_previous_trajpoint,...
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
[ MPOTP_cell, Self_adjustment_Enable_Disable ] = TransitionStrategy(Mode_previous,PosOri_previous, q0q1q2_previous_trajpoint, Mode_current,PosOri_current);

%%
po_Intep = [];
q0q1q2_P2P = [];

for i = 1:length(MPOTP_cell)
    %% Intepotation
    if Mode_previous == 5
        Mode = Mode_current;
    else
        Mode = Mode_previous;
    end
    
    if i ~= 1 && (Mode_current == 3 || Mode_current == 4  || Mode_current == 5 )
        Mode = Mode_current;
    end
        
    [ po_Intep ] =  IntepotationP2P(Mode, PosOri_previous, PosOri_current, NumIntepoPoints, l1, l2);    
    
    %% Iterative IK solution of all intepolation points
    for j = 1:NumIntepoPoints
        %
        if Mode == Mode_current
            PosOri_Effect = PosOri_current;
        elseif Mode == Mode_previous
            PosOri_Effect = PosOri_previous;
        end
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
                elseif Mode == 4
                    q21 = PosOri{8};
                elseif Mode == 5                    
                    if Mode_previous == 5 && Mode_previous == Mode_current
                        q11 = PosOri{7};
                        q21 = PosOri{8};
                    else
                        q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                        q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
                    end
                end
            end
        else
             q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
            q11 = PosOri{7};
            q21 = PosOri{8};
        end
        
        % IK solution 
        [ p, ~, ~, q1q2, ~ ] = IK(Mode, PosOri, q0, q11, q21, q0q1q2_OptimalRow(length(q0q1q2_OptimalRow(:,1)),:), l1, l2);
        q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];
        
        % Optimal Joints Solution
        OptimalJointsSolution;                                             
        q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j,:) = q0q1q2_Optimal_SingleRow;                                            
        %------ Show debugging
        %ReconbotANI(q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,:));
        
        %------ Show Center point of Moving Platform
        %Displacement = [250,250,165.88];
        %p_Base = p(1:3) + Displacement;
        %plot3(p_Base(1),p_Base(2),p_Base(3),'r.');
    end       
    
    if i == 1 && Self_adjustment_Enable_Disable == 1       
        Mode_current = Mode_previous;        
        if Mode == 3        
            PosOri_current{7} = PosOri_previous{7};
            PosOri_current{8} = q0q1q2_OptimalRow(2,7);
        elseif Mode == 4
            PosOri_current{7} = q0q1q2_OptimalRow(2,2);
            PosOri_current{8} = PosOri_previous{8};
        else%if Mode == 5
            PosOri_current{7} = q0q1q2_OptimalRow(2,2);
            PosOri_current{8} = q0q1q2_OptimalRow(2,7);
        end
        PosOri_current = {PosOri_previous{1:6},PosOri_current{7:8}};
        q0q1q2_OptimalRow(1,:) = q0q1q2_OptimalRow(2,:);
    elseif i == 1 && Self_adjustment_Enable_Disable == 2
        PosOri_previous = PosOri_current;
        if Mode == 3        
            q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
        elseif Mode == 4
            q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
            q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
        else
            q0q1q2_OptimalRow(NumIntepoPoints,2) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
            q0q1q2_OptimalRow(NumIntepoPoints,6) = q0q1q2_OptimalRow(NumIntepoPoints - 1,2);
            q0q1q2_OptimalRow(NumIntepoPoints,7) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
            q0q1q2_OptimalRow(NumIntepoPoints,11) = q0q1q2_OptimalRow(NumIntepoPoints - 1,7);
        end        
        PosOri_previous{7} = q0q1q2_OptimalRow(NumIntepoPoints*i,2);
        PosOri_previous{8} = q0q1q2_OptimalRow(NumIntepoPoints*i,7);
    end
    
end

if Self_adjustment_Enable_Disable == 1
    q0q1q2_P2P = [q0q1q2_OptimalRow((NumIntepoPoints+1):2*NumIntepoPoints,:); q0q1q2_OptimalRow(1:NumIntepoPoints,:) ];
elseif Self_adjustment_Enable_Disable == 2
    q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+1,:) = q0q1q2_OptimalRow(NumIntepoPoints*(i-1),:);
    q0q1q2_P2P = [q0q1q2_OptimalRow(1:NumIntepoPoints,:); q0q1q2_OptimalRow((NumIntepoPoints+1):2*NumIntepoPoints,:) ]; 
else
    q0q1q2_P2P = q0q1q2_OptimalRow(1:NumIntepoPoints,:);
end

end
