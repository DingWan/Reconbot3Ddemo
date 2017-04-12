function [ p, EulerAngle_q11_theta, ABC, q1q2, WSvalue ] = IK(Mode, PositionOritationq11q21,inputq0, inputq11, inputq21, q0q1q2_Previous, l1, l2)
%%----------------------- Calculate RCB IK -------------------------
po = PositionOritationq11q21;
q0 = inputq0;
    switch Mode
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
                q11 = inputq11;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                q11 = inputq11;
                q21 = inputq21;
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
                q11 = inputq11;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                q11 = inputq11;
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                obj3T1R = RCB3T1R(PosOri, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            else
                obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
            end
        case 3 % 3T1R-SingularityA1C1
            %We need to intepolate on a cylinder surface
            q11 = inputq11;            
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if Mode == 5 
                %q21 = q0q1q2_mat(n*(IntepPointNum-1)+1,7);
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 ...
                    && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                %q0q1q2_mat(n*(IntepPointNum-1)+i,:) = q0q1q2_mat(n*(IntepPointNum-1)+i-1,:);
                %q21 = q0q1q2_mat(n*(IntepPointNum-1)+i,7);
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
            end
            obj3T1R = RCB3T1RSingularityA1C1(PosOri, q11q12q21q22, l1, l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
        case 4 % 3T1R-SingularityA2C2
            %We need to intepolate on a cylinder surface            
            q21 = inputq21;                      
            q11q12q21q22 = [];
            % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
            % We assume that the precision is 0.02mm (1) as industry manipulators
            % Ci_in_Ob: Ci in frame Ob-xyz
            C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
            C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
            A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
            % %-----------------------------------------------%
            if Mode == 5
                q11 = inputq11;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 ...
                    && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                %q0q1q2_mat(n*(IntepPointNum-1)+i,:) = q0q1q2_mat(n*(IntepPointNum-1)+i-1,:);
                q11 = inputq11;
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
            end
            obj3T1R = RCB3T1RSingularityA2C2(PosOri, q11q12q21q22, l1, l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
        case 5 % 3T1R-SingularityA1C1A2C2
            q11 = inputq11;
            q21 = inputq21;
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
                q11 = inputq11;
                q21 = inputq21;
                PosOri = {po{1}, po{2}, po{3}, [], [], po{6}, q11, q21};
            else
                PosOri = {po{1}, po{2}, po{3}, [], [], po{6}};
            end
            if abs(po{2}) > 1e-12 % y ~= 0
                obj2T2Rsixbar = RCB2T2Rsixbar(PosOri,q11q12q14q23,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                beta_FiveBar = EulerAngle_q11_theta(2);
            elseif abs(po{1}) < 1e-12 && abs(po{2}) < 1e-12% && IntepPointNum ~= 1% x = y = 0                
                obj2T2Rsixbar = RCB2T2Rsixbar(PosOri,q11q12q14q23,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                
            elseif abs(po{1}) > 1e-12 && abs(po{2}) < 1e-12 % y = 0
                po = {po{1}, 0, po{3}, [], beta_FiveBar, 0};
                q11q12q14q22 = [];
                obj2T2Rfivebar = RCB2T2Rfivebar(PosOri,q11q12q14q22,l1,l2);
                [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK;
            end
        case 7 % 2T2R-6-Bar(xy=0)
            % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
            % p = [[], [], z, [], beta, gamma]; x = y = 0
            q11q12q14q23 = [];
            if po{5} == 0 && po{6} == 0
                q11q12q21q22 = [q0q1q2_Previous(2:3),q0q1q2_Previous(7:8)];
                q11q12q14q23 = q11q12q21q22;
                q11 = inputq11;
                q21 = inputq21;
                PosOri = {[], [], po{3}, [], po{5}, po{6}, q11, q21};
            else
                PosOri = {[], [], po{3}, [], po{5}, po{6}};
            end
            obj1T3RRotAroundPoint = RCB1T3RRotAroundPoint(PosOri,q11q12q14q23,l1,l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj1T3RRotAroundPoint.RCB_1T3R_RotAroundPoint_IK;
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
            q11q12q14q23 = [];
            obj2T2Rthreebar = RCB2T2Rthreebar(po,q11q12q14q23,l1,l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rthreebar.RCB_2T2R_ThreeBar_IK;
        case 10 % 2R-SerialA1C1
            %We need to intepolate on shpere surface
            % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y < 0
            q11q12q22q13 = [];
            obj2RserialA1C1 = RCB2RserialA1C1(po,q11q12q22q13,l1,l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
        case 11 % 2R-SerialA2C2
            %We need to intepolate on shpere surface
            % Four-bar linkage with Serial Chain A2C2: [1 1 0 0 0 0]
            % p = [x, y, [], [], [], []]; y > 0
            q11q12q22q13 = [];
            obj2RserialA2C2 = RCB2RserialA2C2(po,q11q12q22q13,l1,l2);
            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
        case 12 % Fixed-SerialA1C1A2C2
            % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
            % p = [0, 0, 0, [], [], []]
            [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
    end    
    %%--------------------------- Method: End -----------------------------
    

end