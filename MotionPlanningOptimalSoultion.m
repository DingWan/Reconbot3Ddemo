% Motion planning and Optimal Solution

%%
n = 20;

% po_current = MPOTP_cell{IntepPointNum};
% po = MPOTP_cell{IntepPointNum};
po_Intep = [];
if IntepPointNum == 1
    q0q1q2_mat(1,:) = q0q1q2;
    po_previous = p_0;
end

for i = 1:length(po)
    if isempty(po{i}) == 1
        p_BinaryCode(i) = 0; 
    else
        p_BinaryCode(i) = 1;
    end
end

% Only because in HomeConfig the q11 and q21 need to be adjusted so as to
% adapt the next step. 
if length(MPOTP_cell{IntepPointNum}) == 8 && length(MPOTP_cell{IntepPointNum+1}) == 6
    % First step: Calculate the next step and get the second row values of q11 and q21 after
    % interpotation, and assign to the previous step.
    po_current = MPOTP_cell{IntepPointNum + 1};
    po = po_current;
    po_previous = p_cell{IntepPointNum};
elseif length(MPOTP_cell{IntepPointNum-1}) == 8 && length(MPOTP_cell{IntepPointNum}) == 6 % IntepPointNum > 2
    % Second step: assign the pre-calculated value q11 and q21 as the end
    % of interpotation values to po{1,7:8}
    po_current = MPOTP_cell{IntepPointNum - 1};
    po_current{1,7} = q0q1q2_mat(n*(IntepPointNum-2)+2,2);
    po_current{1,8} = q0q1q2_mat(n*(IntepPointNum-2)+2,7);
    MPOTP_cell{IntepPointNum - 1} = po_current;
    po = po_current;       
    po_previous = p_cell{IntepPointNum - 1};
else
    po_current = MPOTP_cell{IntepPointNum};
    po = po_current; 
    po_previous = p_cell{IntepPointNum - 1};
end


%=================== Catisian Space Trajctory Planning ====================
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
                po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,2), po_current{i}, n);
            elseif i == 8
                po_Intep(i,:) = linspace(q0q1q2_mat(n*(IntepPointNum-2)+1,7), po_current{i}, n);
            end        
    end
end
%=============================== End ======================================

%%
for i = 1:n

    for j = 1:length(po)
        if isempty(po{j}) ~= 1       
              po{j} = po_Intep(j,i);   
        end
    end
    
    %%----------------------- Calculate RCB IK -------------------------
    if isequal(p_BinaryCode(1:6), [1 1 1 1 0 0]) == 1
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
                if i == 1 && length(po) == 6
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+1,2);
                else
                    q11 = po{7};
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                if i == 1 && length(po) == 6
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+1,7);
                else
                    q21 = po{8};
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                if i == 1 && length(po) == 6
                    q11 = q0q1q2_mat(n*(IntepPointNum-1)+1,2);
                    q21 = q0q1q2_mat(n*(IntepPointNum-1)+1,7);
                else
                    q11 = po{7};
                    q21 = po{8};
                end
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;        
            else
                obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2] = obj3T1R.RCB_3T1R_IK;                
            end
    elseif isequal(p_BinaryCode(1:6), [1 1 1 0 0 1]) == 1
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
                end
                
            if abs(po{2}) > 1e-12 % y ~= 0
                obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                beta_FiveBar = EulerAngle_q11_theta(2);
            elseif abs(po{1}) < 1e-12 && abs(po{2}) < 1e-12 % x = y = 0 
                q0q1q2_mat(i,:) = q0q1q2_mat(i-1,:);
                continue;
            elseif abs(po{2}) < 1e-12 % y = 0 
                po = {po{1}, 0, po{3}, [], beta_FiveBar, 0};
                [~, ~, q1q2] = RCB_2T2R_FiveBar_IK(po, l1, l2);
            end        
    elseif isequal(p_BinaryCode(1:6), [1 1 1 0 1 1]) == 1
        % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
        % p = [x, 0, z, [], beta, 0]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_2T2R_FiveBar_IK(po, l1, l2);
    elseif isequal(p_BinaryCode(1:6), [1 1 1 1 1 0]) == 1
        % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
        % p = [x, 0, z, 0, beta, []]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_ThreeBar_IK(po, l1, l2);
    elseif isequal(p_BinaryCode(1:6), [0 0 1 0 1 1]) == 1
        % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
        % p = [[], [], z, [], beta, gamma]; x = y = 0
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(po, l1, l2);
    elseif isequal(p_BinaryCode(1:6), [1 1 0 0 0 0]) == 1
        if po{2} < 0
            % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y < 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(po, l1, l2);
        elseif po{2} > 0
            % Four-bar linkage with Serial Chain A2C2: [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y > 0
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA2C2_IK(po, l1, l2);
        end
    elseif isequal(p_BinaryCode(1:6), [1 1 1 0 0 0]) == 1
        % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
        % p = [0, 0, 0, [], [], []]
        [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
    end
    %%--------------------------- Method: End -----------------------------
    
    
    %========================== Optimal Solution ========================
    for j = 1:length(q1q2(:,1))
        
        if IntepPointNum == 1 && i == 1
            q0q1q2_mat(i,:) = q0q1q2(1:11);
            q1q2A1C1_norm(j) = norm(q1q2(j,1:5) - q0q1q2_mat(i,2:6));
            q1q2A2C2_norm(j) = norm(q1q2(j,6:10) - q0q1q2_mat(i,7:11));
        else
            q1q2A1C1_norm(j) = norm(q1q2(j,1:5) - q0q1q2_mat(n*(IntepPointNum-1)+i-1,2:6));
            q1q2A2C2_norm(j) = norm(q1q2(j,6:10) - q0q1q2_mat(n*(IntepPointNum-1)+i-1,7:11));
        end
        
    end
    [rowsA1C1,colsA1C1] = find(q1q2A1C1_norm == min(min(q1q2A1C1_norm)));
    [rowsA2C2,colsA2C2] = find(q1q2A2C2_norm == min(min(q1q2A2C2_norm)));
    %
    SolutionRow_A1C1 = colsA1C1(1);
    SolutionRow_A2C2 = colsA2C2(1);

    q0q1q2_mat(n*(IntepPointNum-1)+i,:) = [q0, q1q2(SolutionRow_A1C1,1:5), q1q2(SolutionRow_A2C2,6:10)];
    %============================= End =================================    
    
    %========================== Collision Check ============================
    [CollisionOccur_DistanceA1C1(n*(IntepPointNum-1)+i,:), CollisionPoints_A1C1, CollisionOccur_DistanceA2C2(n*(IntepPointNum-1)+i,:), CollisionPoints_A2C2]...
        = CollisionCheck(q0q1q2_mat(n*(IntepPointNum-1)+i,:)*180/pi);    
    %============================= End =================================
    
     %========================== Collision Line ============================
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