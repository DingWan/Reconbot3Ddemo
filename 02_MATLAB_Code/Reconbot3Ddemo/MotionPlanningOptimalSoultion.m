% Motion planning and Optimal Solution

%%
n = 50;

% po_current = MPOTP_cell{IntepPointNum};
% po = MPOTP_cell{IntepPointNum};
po_Intep = [];
if IntepPointNum == 1
    q0q1q2_mat(1,:) = q0q1q2_cell{1,:};
    po_previous = p_0;
end

for i = 1:length(po)
    if isempty(po{i}) == 1
        p_BinaryCode(i) = 0; 
    else
        p_BinaryCode(i) = 1;
    end
end

% In case the current step is 2T2R_Fivebar and previous step is
% 2T2R_Sixbar, Then, force the 2T2R_Fivebar mode (8) equals to Mode = 6
if MPOTP_cell{IntepPointNum}{1} == 8 && MPOTP_cell{IntepPointNum - 1}{1} == 6
    MPOTP_cell{IntepPointNum}{1} = 6;
    MPOTP_cell{IntepPointNum}{7} = [];
end

if IntepPointNum == 1 
    Self_adjustment_Enable = 1;    
end

%% =================== Mode 3T1R-SingularityA1C1A2C2 to any Mode ====================
if Self_adjustment_Enable == 1
   % First step: Calculate the next step and get the second row values of q11 and q21 after
    % interpotation, and assign to the previous step.
    Mode_current = MPOTP_cell{IntepPointNum + 1}{1};
    po_current = {  MPOTP_cell{IntepPointNum + 1}{2:length(MPOTP_cell{IntepPointNum + 1})} };
    po = po_current;
    po_previous = p_cell{IntepPointNum};
    Self_adjustment_Enable = 2;
elseif Self_adjustment_Enable == 2 && MPOTP_cell{IntepPointNum}{1} ~= 5  
    % Second step: assign the pre-calculated value q11 and q21 as the end
    % of interpotation values to po{1,7:8}
    Mode_current = MPOTP_cell{IntepPointNum - 1}{1};
    po_current = {  MPOTP_cell{IntepPointNum - 1}{2:length(MPOTP_cell{IntepPointNum - 1})}  };
    if MPOTP_cell{IntepPointNum - 1}{1} == 5
        po_current{1,7} = q0q1q2_mat(n*(IntepPointNum-2)+2,2);
        po_current{1,8} = q0q1q2_mat(n*(IntepPointNum-2)+2,7);
    elseif MPOTP_cell{IntepPointNum - 1}{1} == 3
        po_current{1,7} = q0q1q2_mat(n*(IntepPointNum-1),2);
    elseif MPOTP_cell{IntepPointNum - 1}{1} == 4
        po_current{1,8} = q0q1q2_mat(n*(IntepPointNum-1),7);
    end
    MPOTP_cell{IntepPointNum - 1} = { MPOTP_cell{IntepPointNum - 1}{1}, po_current{:} };
    po = po_current;       
    po_previous = p_cell{IntepPointNum - 1};
    Self_adjustment_Enable = 0;
else
    Self_adjustment_Enable = 0;
    if MPOTP_cell{IntepPointNum}{1} == 5
        if  IntepPointNum == 1 || MPOTP_cell{IntepPointNum - 1}{1} == 5 && IntepPointNum + 1 <= length(MPOTP_cell)
            Mode_current = MPOTP_cell{IntepPointNum + 1}{1};
            po_current = {  MPOTP_cell{IntepPointNum  + 1}{2:length(MPOTP_cell{IntepPointNum  + 1})}  };
            po = po_current;
            po_previous = p_cell{IntepPointNum};
        else
            for i = 1:length(q0q1q2_mat(n*(IntepPointNum-1),:))
                q0q1q2_mat(n*(IntepPointNum-1)+1:n*IntepPointNum,i) = linspace(q0q1q2_mat(n*(IntepPointNum-1),i),q0q1q2_mat(1,i), n);
            end      
            %break
        end
    else
        if  (MPOTP_cell{IntepPointNum - 1}{1} == 3 && MPOTP_cell{IntepPointNum}{1} == 3) ...
                || (MPOTP_cell{IntepPointNum - 1}{1} == 4 && MPOTP_cell{IntepPointNum}{1} == 4)
            if MPOTP_cell{IntepPointNum - 1}{1} == 3
                po_current{1,7} = q0q1q2_mat(n*(IntepPointNum-1),2);
            elseif MPOTP_cell{IntepPointNum - 1}{1} == 4
                po_current{1,8} = q0q1q2_mat(n*(IntepPointNum-1),7);
            end
        end
        % both steps are non-singularity
        Mode_current = MPOTP_cell{IntepPointNum }{1};
        po_current = {  MPOTP_cell{IntepPointNum}{2:length(MPOTP_cell{IntepPointNum})}  };
        po = po_current;
        po_previous = p_cell{IntepPointNum - 1};
    end
end
%=============================== End ======================================


%% =================== Catisian Space Trajctory Planning ====================
if MPOTP_cell{IntepPointNum}{1} == 5 && IntepPointNum + 1 <= length(MPOTP_cell) && MPOTP_cell{IntepPointNum + 1}{1} == 5
    for i = 1:length(po)
        % Here uses to make sure the start singular configuration can go to the
        % selected first mode by adjusting q11 and q21
        if isempty(po{i}) == 1
            po_Intep(i,:) = 0;
        else
            if i < 7
                po_Intep(i,:) = linspace(po_previous(i),po_current{i}, n);
            elseif i == 7
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,2), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),2), po_current{i}, n);
                end
            elseif i == 8
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,7), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),7), po_current{i}, n);
                end
            end
        end
    end
elseif (MPOTP_cell{IntepPointNum}{1} == 5 || MPOTP_cell{IntepPointNum}{1} == 3) && IntepPointNum + 1 <= length(MPOTP_cell) && MPOTP_cell{IntepPointNum + 1}{1} == 3
     for i = 1:length(po)
        % Here uses to make sure the start singular configuration can go to the
        % selected first mode by adjusting q11 and q21
        if isempty(po{i}) == 1
            po_Intep(i,:) = 0;
        else
            if i < 7
                po_Intep(i,:) = linspace(po_previous(i),po_current{i}, n);
            elseif i == 7
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,2), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),2), po_current{i}, n);
                end
            elseif i == 8
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,7), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),7), po_current{i}, n);
                end
            end
        end
     end     
     po_Intep(1,:) = -l1/2 * sin(po_Intep(4,:));
     po_Intep(2,:) = l1/2 * (cos(po_Intep(4,:)) - 1);     
elseif (MPOTP_cell{IntepPointNum}{1} == 5  || MPOTP_cell{IntepPointNum}{1} == 4) && IntepPointNum + 1 <= length(MPOTP_cell) && MPOTP_cell{IntepPointNum + 1}{1} == 4
     for i = 1:length(po)
        % Here uses to make sure the start singular configuration can go to the
        % selected first mode by adjusting q11 and q21
        if isempty(po{i}) == 1
            po_Intep(i,:) = 0;
        else
            if i < 7
                po_Intep(i,:) = linspace(po_previous(i),po_current{i}, n);
            elseif i == 7
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,2), po_current{i}, n);
                elseif MPOTP_cell{IntepPointNum - 1}{1} == 5
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,2), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),2), po_current{i}, n);
                end
            elseif i == 8
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,7), po_current{i}, n);
                elseif MPOTP_cell{IntepPointNum - 1}{1} == 5
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,7), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),7), po_current{i}, n);
                end
            end
        end
     end     
     po_Intep(1,:) = l1/2 * sin(po_Intep(4,:));
     po_Intep(2,:) = l1/2 * (- cos(po_Intep(4,:)) + 1);    
else
    for i = 1:length(po)
        % Here uses to make sure the start singular configuration can go to the
        % selected first mode by adjusting q11 and q21
        if isempty(po{i}) == 1
            p_BinaryCode(i) = 0;
            po_Intep(i,:) = 0;
        else
            p_BinaryCode(i) = 1;
            if i < 7
                po_Intep(i,:) = linspace(po_previous(i),po_current{i}, n);
            elseif i == 7
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,2), po_current{i}, n);
                elseif Mode_current == 5
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,2), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),2), po_current{i}, n);
                end
            elseif i == 8
                if IntepPointNum == 1
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1)+1,7), po_current{i}, n);
                elseif Mode_current == 5
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,7), po_current{i}, n);
                else
                    po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-1),7), po_current{i}, n);
                end
            end
        end
    end
end
%=============================== End ======================================

%% Execut IK for each point and also check the collision
for i = 1:n

    for j = 1:length(po)
        if isempty(po{j}) ~= 1       
              po{j} = po_Intep(j,i);   
        end
    end
    
    %%----------------------- Calculate RCB IK -------------------------
    switch Mode_current
        case 1 % 3T2R = 3T1R
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs((C2_in_Ob(1) - A2(1)) > 1e-2 || abs(C2_in_Ob(2) - A2(2)) > 1e-2)
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+i,2);
                else
                    if laststep ~= 0
                        q11 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,2);
                    else
                        q11 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,2);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                else
                    if laststep ~= 0
                        q21 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,7);
                    else
                        q21 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,7);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+i,2);
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                else
                    if laststep ~= 0
                        q11 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,2);
                        q21 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,7);
                    else
                        q11 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,2);
                        q21 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,7);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            else
                obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            end
        case 2 % 3T1R
            %We need to intepolate 3D space
            % 3T1R mode:  [1 1 1 1 0 0]
            % p = [x, y, z, alpha, [], []];
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs((C2_in_Ob(1) - A2(1)) > 1e-2 || abs(C2_in_Ob(2) - A2(2)) > 1e-2)
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+i,2);
                else
                    if laststep ~= 0
                        q11 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,2);
                    else
                        q11 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,2);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                else
                    if laststep ~= 0
                        q21 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,7);
                    else
                        q21 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,7);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                if Mode_current ~= MPOTP_cell{IntepPointNum}{1}
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+i,2);
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                else
                    if laststep ~= 0
                        q11 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,2);
                        q21 = q0q1q2_mat(n*(IntepPointNum-1)+i-1,7);
                    else
                        q11 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,2);
                        q21 = q0q1q2_mat(n*(IntepPointNum-2)+i-1,7);
                    end
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            else
                obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            end
        case 3 % 3T1R-SingularityA1C1
            %We need to intepolate on a cylinder surface
            q11 = po_Intep(7,i);            
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if MPOTP_cell{IntepPointNum}{1} == 5 && i == 1
                q21 = q0q1q2_mat(n*(IntepPointNum-1)+1,7);
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 ...
                    && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                q0q1q2_mat(n*(IntepPointNum-1)+i,:) = q0q1q2_mat(n*(IntepPointNum-1)+i-1,:);
                q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
            end
            obj3T1R = RCB3T1RSingularityA1C1(PosOri, q11q12q21q22, l1, l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
        case 4 % 3T1R-SingularityA2C2
            %We need to intepolate on a cylinder surface            
            q21 = po_Intep(8,i);                      
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if MPOTP_cell{IntepPointNum}{1} == 5 && i == 1 ...
                q11 = q0q1q2_mat(n*(IntepPointNum-1)+1,2);
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 ...
                    && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                q0q1q2_mat(n*(IntepPointNum-1)+i,:) = q0q1q2_mat(n*(IntepPointNum-1)+i-1,:);
                q11 = q0q1q2_mat(n*(IntepPointNum-1)+i,2);
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
            end
            obj3T1R = RCB3T1RSingularityA2C2(PosOri, q11q12q21q22, l1, l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
        case 5 % 3T1R-SingularityA1C1A2C2
            q11 = po_Intep(7,i);
            q21 = po_Intep(8,i);
            q11q12q21q22 = [];
            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            obj3T1R = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q22, l1, l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1A2C2_IK;
        case 6 % 2T2R-6-Bar
            %We need to intepolate 3D space
            % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
            % p = [x, y, z, [], [], gamma]
            q11q12q14q23 = [];
            % Judge the start point
            % We assume that the precision is 0.02mm (1) as industry manipulators
            if po{1} == 0 && po{2} == 0
                if i == 1 && length(po) == 6
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+1,2);
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+1,7);
                else
                    q11 = po{7};
                    q21 = po{8};
                end
                PosOri = {po{1}, po{2}, po{3}, [], [], po{6}, q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, [], [], po{6}};
            end
            if abs(po{2}) > 1e-12 % y ~= 0
                obj2T2Rsixbar = RCB2T2Rsixbar(PosOri,q11q12q14q23,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                beta_FiveBar = EulerAngle_q11_theta(2);
            elseif abs(po{1}) < 1e-12 && abs(po{2}) < 1e-12% && IntepPointNum ~= 1% x = y = 0                
                if IntepPointNum == 1
                    q0q1q2_mat(n*(IntepPointNum-1) + i + 1,:) = q0q1q2_mat(n*(IntepPointNum-1) + i,:);
                else
                    q0q1q2_mat(n*(IntepPointNum-1) + i,:) = q0q1q2_mat(n*(IntepPointNum-1) + i - 1,:);
                end
            elseif abs(po{1}) > 1e-12 && abs(po{2}) < 1e-12 % y = 0
                po = {po{1}, 0, po{3}, [], beta_FiveBar, 0};
                q11q12q14q22 = [];
                obj2T2Rfivebar = RCB2T2Rfivebar(PosOri,q11q12q14q22,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK;
            end
        case 7 % 2T2R-6-Bar(xy=0)
            % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
            % p = [[], [], z, [], beta, gamma]; x = y = 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(po, l1, l2);
        case 8 % 2T2R-5-Bar
            %We need to intepolate on o-xz plane
            % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
            % p = [x, 0, z, [], beta, 0]
            q11q12q14q22 = [];
            obj2T2Rfivebar = RCB2T2Rfivebar(po,q11q12q14q22,l1,l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK;
        case 9 % 2T1R-3-BarSerial
            %We need to intepolate on o-xz plane
            % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
            % p = [x, 0, z, 0, beta, []]
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_ThreeBar_IK(po, l1, l2);
        case 10 % 2R-SerialA1C1
            %We need to intepolate on shpere surface
            % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y < 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(po, l1, l2);
        case 11 % 2R-SerialA2C2
            %We need to intepolate on shpere surface
            % Four-bar linkage with Serial Chain A2C2: [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y > 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA2C2_IK(po, l1, l2);
        case 12 % Fixed-SerialA1C1A2C2
            % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
            % p = [0, 0, 0, [], [], []]
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
    end    
    %%--------------------------- Method: End -----------------------------
    
    
    %========================== Optimal Solution ========================
    q1q2A1C1_norm = [];
    q1q2A2C2_norm = [];
    q1q2A1C1A1C1_norm = [];
    for j = 1:length(q1q2(:,1))
        
        if Mode_current ~= MPOTP_cell{IntepPointNum}{1} && (Mode_current == 5 || MPOTP_cell{IntepPointNum}{1} == 5) && i == 1
            % we first confirm the one value of the end point (q0q1q2_required_endpoint) by comparing it with Homeconfiguration
            if Mode_current ~= 5
                q0q1q2_matrix_start = q0q1q2_cell{IntepPointNum};
                q0q1q2_matrix_end = q0q1q2_cell{IntepPointNum + 1};
                %========= Here we must judge five-bar or three-bar sparately;=====
                % Because Five/Three-bar should be judge as whole, and other mode should judge sparate
                if Mode_current == 8 || Mode_current == 9
                    for k = 1:length(q0q1q2_matrix_end(:,1))
                        q1q2_matrix_norm(k) = norm(q0q1q2_matrix_end(k,2:11) - q0q1q2_matrix_start(1,2:11));
                    end
                    [rowsq1q2,colsq1q2] = find(q1q2_matrix_norm == min(min(q1q2_matrix_norm)));
                    SolutionRow_q1q2 = colsq1q2(1);
                    q0q1q2_required_endpoint = [q0q1q2_matrix_end(1), q0q1q2_matrix_end(SolutionRow_q1q2,2:11)];
                else
                    for k = 1:length(q0q1q2_matrix_end(:,1))
                        q1_matrix_norm(k) = norm(q0q1q2_matrix_end(k,2:6) - q0q1q2_matrix_start(1,2:6));
                        q2_matrix_norm(k) = norm(q0q1q2_matrix_end(k,7:11) - q0q1q2_matrix_start(1,7:11));
                    end
                    [rowsq1,colsq1] = find(q1_matrix_norm == min(min(q1_matrix_norm)));
                    [rowsq2,colsq2] = find(q2_matrix_norm == min(min(q2_matrix_norm)));
                    SolutionRow_q1 = colsq1(1);
                    SolutionRow_q2 = colsq2(1);
                    q0q1q2_required_endpoint = [q0q1q2_matrix_end(1), q0q1q2_matrix_end(SolutionRow_q1,2:6), q0q1q2_matrix_end(SolutionRow_q2,7:11)];
                end
                %==================================================================
            end
            if MPOTP_cell{IntepPointNum}{1} == 5% 3T1R-SingularityA1C1A2C2
                q0q1q2_mat(i,:) = q0q1q2_cell{IntepPointNum,:};
            elseif MPOTP_cell{IntepPointNum}{1} == 3 || MPOTP_cell{IntepPointNum}{1} == 4
                q0q1q2_mat(n*(IntepPointNum-1)+1,2:11) = q0q1q2_mat(n*(IntepPointNum-2)+1,2:11);
            elseif MPOTP_cell{IntepPointNum}{1} < 3 || MPOTP_cell{IntepPointNum}{1} > 5 % other mode
                q0q1q2_mat(n*(IntepPointNum-1)+1,2:11) = q0q1q2_mat(n*(IntepPointNum-2)+1,2:11);
            end
        elseif Mode_current ~= MPOTP_cell{IntepPointNum}{1} && (Mode_current == 5 || MPOTP_cell{IntepPointNum}{1} == 5) && i == 2 && Mode_current ~= 5
            % Secondly, confirm the second value that has minimum norm value with the end point (q0q1q2_required_endpoint)
            %========= Here we must judge five-bar or three-bar sparately;=====
            if Mode_current == 8 || Mode_current == 9
                q1q2A1C1A2C2_norm(j) = norm(q1q2(j,1:10) - q0q1q2_required_endpoint(2:11));
            else
                q1q2A1C1_norm(j) = norm(q1q2(j,1:5) - q0q1q2_required_endpoint(2:6));
                q1q2A2C2_norm(j) = norm(q1q2(j,6:10) - q0q1q2_required_endpoint(7:11));
            end
            %==================================================================
        else
            % Thirdly, confirm the following value that has minimum norm value with the last step value (q0q1q2_mat(LastOneStep))
            if Mode_current == MPOTP_cell{IntepPointNum}{1} && i == 1
                if Mode_current == 5
                    if MPOTP_cell{IntepPointNum}{1} == 5 && IntepPointNum ~= 1
                        q0q1q2_mat(n*(IntepPointNum-1)+1,2:11) = q0q1q2_mat(n*(IntepPointNum-1),2:11);
                    else
                        q0q1q2_mat(i,:) = q0q1q2_cell{IntepPointNum,:};
                    end
                else
                    q0q1q2_mat(n*(IntepPointNum-1)+1,2:11) = q0q1q2_mat(n*(IntepPointNum-1),2:11);
                end
            else
                %========= Here we must judge five-bar or three-bar sparately;=====
                if Mode_current == 8 || Mode_current == 9
                    q1q2A1C1A2C2_norm(j) = norm(q1q2(j,1:10) - q0q1q2_mat(n*(IntepPointNum-1)+i-1,2:11));
                else
                    q1q2A1C1_norm(j) = norm(q1q2(j,1:5) - q0q1q2_mat(n*(IntepPointNum-1)+i-1,2:6));
                    q1q2A2C2_norm(j) = norm(q1q2(j,6:10) - q0q1q2_mat(n*(IntepPointNum-1)+i-1,7:11));
                end
                %==================================================================
            end
        end
        
    end
    
    if  i == 1 && (Mode_current == 5 || MPOTP_cell{IntepPointNum}{1} == 5) 
        %%%%% If the step it's self-adjustment,the first step is Homeconfiguration
        %%%%%  But we should also consider if it's not the first step, and it's other steps, it should be also possible to handle it
    else
        if Mode_current == MPOTP_cell{IntepPointNum}{1} && i == 1
            %%%%% if the two steps are the same, so, this step is the same as the last step
        else
            if Mode_current == 8 || Mode_current == 9
                [rowsA1C1A2C2,colsA1C1A2C2] = find(q1q2A1C1A2C2_norm == min(min(q1q2A1C1A2C2_norm)));
                SolutionRow_A1C1A2C2 = colsA1C1A2C2(1);
                q0q1q2_mat(n*(IntepPointNum-1)+i,:) = [q0, q1q2(SolutionRow_A1C1A2C2,1:10)];
            else
                [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
                [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
                SolutionRow_A1C1 = colsA1C1(1);
                SolutionRow_A2C2 = colsA2C2(1);
                q0q1q2_mat(n*(IntepPointNum-1)+i,:) = [q0, q1q2(SolutionRow_A1C1,1:5), q1q2(SolutionRow_A2C2,6:10)];
            end
        end
    end
    %============================= End =================================    
    
    %========================== Collision Check ============================
    [CollisionOccur_DistanceA1C1(n*(IntepPointNum-1)+i,:), CollisionPoints_A1C1, CollisionOccur_DistanceA2C2(n*(IntepPointNum-1)+i,:), CollisionPoints_A2C2]...
        = CollisionCheck(q0q1q2_mat(n*(IntepPointNum-1)+i,:)*180/pi);    
    %============================= End =================================
    
     %========================== Collision Line ============================
     % In file "InitHome.m", we use "setappdata(obj,name,val)" to set obj = 0; 
     % L1 = patch('faces', BaseLow_data.F1, 'vertices' ,Link_BaseLow(:,1:3));
     % Li = ... (i = 1-11, represent all links)
     % Tr = plot3(0,0,0,'b.'); % holder for trail paths
     % CPsA1C1 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A1C1
     % CPsA2C2 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A2C2
     % setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,Tr,CPsA1C1,CPsA2C2])
     
     % setappdata(0,'xCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
     % setappdata(0,'yCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
     % setappdata(0,'zCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.

     % setappdata(0,'xCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
     % setappdata(0,'yCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
     % setappdata(0,'zCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
 
    handles = getappdata(0,'patch_h');
    CPsA1C1 = handles(13);
    CPsA2C2 = handles(14);
    CollisionDetection = 'y';
    if CollisionDetection == 'y'
        x_CollisionPoints_A1C1 = getappdata(0,'xCollisionPoints_A1C1');
        y_CollisionPoints_A1C1 = getappdata(0,'yCollisionPoints_A1C1');
        z_CollisionPoints_A1C1 = getappdata(0,'zCollisionPoints_A1C1');
        %
        x_CollisionPoints_A2C2 = getappdata(0,'xCollisionPoints_A2C2');
        y_CollisionPoints_A2C2 = getappdata(0,'yCollisionPoints_A2C2');
        z_CollisionPoints_A2C2 = getappdata(0,'zCollisionPoints_A2C2');
        %
        xCPsA1C1data = [x_CollisionPoints_A1C1 CollisionPoints_A1C1(:,1)];
        yCPsA1C1data = [y_CollisionPoints_A1C1 CollisionPoints_A1C1(:,2)];
        zCPsA1C1data = [z_CollisionPoints_A1C1 CollisionPoints_A1C1(:,3)];
        %
        xCPsA2C2data = [x_CollisionPoints_A2C2 CollisionPoints_A2C2(:,1)];
        yCPsA2C2data = [y_CollisionPoints_A2C2 CollisionPoints_A2C2(:,2)];
        zCPsA2C2data = [z_CollisionPoints_A2C2 CollisionPoints_A2C2(:,3)];
        %
        setappdata(0,'xCollisionPoints_A1C1',xCPsA1C1data); % used for trail tracking.
        setappdata(0,'yCollisionPoints_A1C1',yCPsA1C1data); % used for trail tracking.
        setappdata(0,'zCollisionPoints_A1C1',zCPsA1C1data); % used for trail tracking.
        %
        setappdata(0,'xCollisionPoints_A2C2',xCPsA2C2data); % used for trail tracking.
        setappdata(0,'yCollisionPoints_A2C2',yCPsA2C2data); % used for trail tracking.
        setappdata(0,'zCollisionPoints_A2C2',zCPsA2C2data); % used for trail tracking.
        %
        set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-'); %hold off
        set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-'); %hold off
    end
    %============================= End =================================
    
    %================== Center Point of Moving Platform ===================
%     ReconbotANI(q0q1q2_mat(n*(IntepPointNum-1)+i,:));
    Displacement = [250,250,165.88];
    p = p(1:3) + Displacement;
    handles = getappdata(0,'patch_h');
    Tr = handles(12);
    trail = 'y';
    if trail == 'y'
        x_trail = getappdata(0,'xtrail');
        y_trail = getappdata(0,'ytrail');
        z_trail = getappdata(0,'ztrail');
        %
        xTrdata = [x_trail p(1)];
        yTrdata = [y_trail p(2)];
        zTrdata = [z_trail p(3)];
        %
        setappdata(0,'xtrail',xTrdata); % used for trail tracking.
        setappdata(0,'ytrail',yTrdata); % used for trail tracking.
        setappdata(0,'ztrail',zTrdata); % used for trail tracking.
        %
        set(Tr,'xdata',xTrdata,'ydata',yTrdata,'zdata',zTrdata);
    end
    %plot3(p(1),p(2),p(3),'.r');
    %============================= End =================================

    %========================== Animation ============================
    %ReconbotANI(q0q1q2_mat(n*(IntepPointNum-1)+i,:));
    %============================ End ================================
    
end