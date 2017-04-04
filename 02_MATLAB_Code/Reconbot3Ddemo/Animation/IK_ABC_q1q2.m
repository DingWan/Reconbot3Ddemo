 %%  --------------------------------- Method I: Start ---------------------------------------
%             str = {'3T1R';'2T2R-6-Bar';'2T2R-6-Bar(xy=0)';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';'2R-SerialA2C2';'Fixed-SerialA1C1A2C2';};
%             [s,v] = listdlg('PromptString','Select a Operation Mode:','SelectionMode','single',...
%                 'ListString',str);
%             if v == 0
%                 break;
%             end
            
            switch s
                case 1
                    %%------------------------ 3T1R IK ------------------------
                    % 3T1R mode:  [1 1 1 1 0 0]
                    % p = [x, y, z, alpha, [], []];
                    po_cell = inputdlg({'x ([-L1/2,L1/2])','y ([-L1/2,L1/2])','z(z>0)','alpha ([0,pi/2])'},'3T1R', [1 20; 1 20; 1 20; 1 20]);
                    for i=1:3 po_num(i) = str2num(po_cell{i}); end
                    po_num(4) = str2num(po_cell{4}) * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0 && po_num(3) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), po_num(3), po_num(4), [], []};
                        q11q12q21q22 = [];
                        obj3T1R = RCB3T1R(po,q11q12q21q22,l1,l2);
                        [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
                    end
                case 2
                    %%------------------------ 2T2R IK: Six-bar ------------------------
                    % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
                    % p = [x, y, z, [], [], gamma]
                    po_cell = inputdlg({'x (-L2,L2)','y (-L2,L2)','z ([0, 2*L2])','gamma ([0,pi/2])'},'2T2R-6-Bar', [1 20; 1 20; 1 20; 1 20;]);
                    for i=1:3 po_num(i) = str2num(po_cell{i}); end
                    po_num(4) = str2num(po_cell{4}) * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0
                        if po_num(3) == 0
                            po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                        else
                            % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
                            % p = [[], [], z, [], beta, gamma]; x = y = 0
                            po_cell = inputdlg({'beta ([0,pi/2])'},'2T2R-6-Bar', [1 20]);
                            po_num(5) = str2num(po_cell{1});
                            po = {0, 0, po_num(3), [], po_num(5), po_num(4)};
                            [EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(po, l1, l2);
                        end
                    else
                        po = {po_num(1), po_num(2), po_num(3), [], [], po_num(4)};
                        q11q12q14q23 = [];
                        obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
                        [p, EulerAngle_q11_theta, ABC, q1q2] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                    end
                case 3
                    %%------------------------ 2T2R IK£ºRotate around point  ------------------------
                    % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
                    % p = [[], [], z, [], beta, gamma]; x = y = 0
                    po_cell = inputdlg({'z ([0, 2*L2])','beta ([0,pi/2])','gamma ([0,pi/2])'},'2T2R-6-Bar', [1 20; 1 20; 1 20;]);
                    for i=2:3 po_num(i) = str2num(po_cell{i}) * pi / 180; end
                    po_num(1) = str2num(po_cell{1});
                    po = {[], [], po_num(1), [], po_num(2), po_num(3)};
                    [EulerAngle_q11_theta, ABC, q1q2] = RCB_1T2R_RotAroundPoint_IK(po, l1, l2);
                case 4
                    %%------------------------ 2T1R IK£ºPlanar Five-bar ------------------------
                    % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
                    % p = [x, 0, z, [], beta, 0]
                    po_cell = inputdlg({'x (-L2,L2)','z ([0, 2*L2])','beta ([0,pi/2])'},'2T2R-5-Bar', [1 20; 1 20; 1 20;]);
                    for i=1:2 po_num(i) = str2num(po_cell{i}); end
                    po_num(3) = str2num(po_cell{3}) * pi / 180;
                    po = {po_num(1), 0, po_num(2), [], po_num(3), 0};
                    [EulerAngle_q11_theta, ABC, q1q2] = RCB_2T2R_FiveBar_IK(po, l1, l2);
                case 5
                    %%------------------------- 2T1R IK£ºPlanar Three-bar -----------------------
                    % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
                    % p = [x, 0, z, 0, beta, []]
                    po_cell = inputdlg({'x (-L2,L2)','z ([0, 2*L2])','beta ([0,pi/2])'},'2T1R-3-BarSerial', [1 20; 1 20; 1 20;]);
                    for i=1:2 po_num(i) = str2num(po_cell{i}); end
                    po_num(3) = str2num(po_cell{3}) * pi / 180;
                    po = {po_num(1), 0, po_num(2), 0, po_num(3), []};
                    [EulerAngle_q11_theta, ABC, q1q2] = RCB_ThreeBar_IK(po, l1, l2);
                case 6
                    %-------------------------- Branch Chain A1C1 --------------------------------
                    % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                    % p = {x, y, [], [], [], []}; y < 0
                    po_cell = inputdlg({'x ([-L1/2,L1/2])','y ([-L1/2,0])', 'theta12'},'2R-SerialA1C1', [1 20; 1 20; 1 20]);
                    for i=1:2 po_num(i) = str2num(po_cell{i}); end
                    po_num(3) = str2num(po_cell{3}) * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), [], [], [], [], po_num(3)};
                        [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA1C1_IK(po, l1, l2);
                    end
                case 7
                    %-------------------------- Branch Chain A2C2 --------------------------------
                    % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                    % p = {x, y, [], [], [], []}; y > 0
                    po_cell = inputdlg({'x ([-L1/2,L1/2])','y ([0,L1/2])','theta22'},'2R-SerialA1C1', [1 20; 1 20; 1 20]);
                    for i=1:2 po_num(i) = str2num(po_cell{i}); end
                    po_num(3) = str2num(po_cell{3}) * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), [], [], [], [], po_num(3)};
                        [EulerAngle_q11_theta, ABC, q1q2] = RCB_2R_SerialA2C2_IK(po, l1, l2);
                    end
                case 8
                    %%------------------------- Two Serial Chains -----------------------------
                    % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
                    % p = {0, 0, 0, [], [], []}
                    po_cell = inputdlg({'theta11 ([-pi, pi])','theta12 ([-pi/4, 5*pi/4])','theta21 ([-pi, pi])','theta22 ([-pi/4, 5*pi/4])'},'Fixed-SerialA1C1A2C2', [1 20; 1 20; 1 20; 1 20]);
                    for i=1:4 po_num(i) = str2num(po_cell{i}) * pi / 180; end
                    po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
            end
            %
            % Judge if mechanism recovery to fixed mode
            if length(po) == 10 && s ~= 8
                po_cell = inputdlg({'theta11 ([-pi, pi])','theta12 ([-pi/4, 5*pi/4])','theta21 ([-pi, pi])','theta22 ([-pi/4, 5*pi/4])'},'Fixed-SerialA1C1A2C2', [1 20; 1 20; 1 20; 1 20]);
                for i=1:4 po_num(i) = str2num(po_cell{i}) * pi / 180; end
                po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                [EulerAngle_q11_theta, ABC, q1q2] = RCB_FixedSerialChain_IK(po, l1, l2);
            end
            %%------------------------------ Method I: End ----------------------------------------