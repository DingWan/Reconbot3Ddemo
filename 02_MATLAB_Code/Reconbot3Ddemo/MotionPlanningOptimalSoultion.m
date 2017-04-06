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
    [ po_Intep ] =  IntepotationP2P(Mode_current, PosOri_previous, PosOri_current, NumIntepoPoints);    
    
    %% Iterative IK solution of all intepolation points
    for j = 1:NumIntepoPoints
        %
        for k = 1:length(PosOri_current)
            if isempty(PosOri_current{k}) ~= 1
                PosOri{k} = po_Intep(k,j);
            else
                PosOri{k} = [];
            end
        end
        
        % We assign the q11 and q21 with previous one step value
        if i == 1
            if j == 1
                q0 = q0q1q2_previous_trajpoint(1);
                q11 = q0q1q2_previous_trajpoint(2);
                q21 = q0q1q2_previous_trajpoint(7);
                q0q1q2_OptimalRow(1,:) = q0q1q2_previous_trajpoint;
            else
                q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
                q11 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,2);
                q21 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,7);
            end
        else
             q0 = q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j-1,1);
            q11 = PosOri{7};
            q21 = PosOri{8};
        end
        
        % IK solution 
        [ p, ~, ~, q1q2, WSvalue ] = IK(Mode_current, PosOri, q0, q11, q21, l1, l2);
        q0q1q2_CurrentStep = [zeros(length(q1q2(:,1)),1),q1q2];
        
        % Optimal Joints Solution
        OptimalJointsSolution;                                             
        q0q1q2_OptimalRow(NumIntepoPoints*(i-1)+j,:) = q0q1q2_Optimal_SingleRow;                                            
        
    end
    
    %q0q1q2_P2P = [q0q1q2_P2P; q0q1q2_OptimalRow(i,:,:)];    
    
    if Self_adjustment_Enable_Disable == 1       
        Mode_current = Mode_previous;
        PosOri_current = PosOri_previous;
        PosOri_current{7} = q0q1q2_OptimalRow(2,2);
        PosOri_current{8} = q0q1q2_OptimalRow(2,7);
        %----di yi bu 21 ti huan
    end
    
end

if Self_adjustment_Enable_Disable == 1
    q0q1q2_P2P = [q0q1q2_OptimalRow((NumIntepoPoints+1):2*NumIntepoPoints,:); q0q1q2_OptimalRow(1:NumIntepoPoints,:) ];
end


end

function c = exchange(a, b)
c = [b; a];
end