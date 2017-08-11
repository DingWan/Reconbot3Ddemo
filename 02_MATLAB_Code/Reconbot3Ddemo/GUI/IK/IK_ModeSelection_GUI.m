 
%%
    q0 = 0; 
    q0q1q2_0 = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
    p_0 = [0 0 208.879343162506 0 0 0, 0 0];
%%

switch Mode
                case 1 % 3T2R
                    %%------------------------ 3T1R IK ------------------------
                    % 3T1R mode:  [1 1 1 1 0 0]
                    % p = [x, y, z, alpha, [], []];
                    po_num(1)=x; po_num(2)=y; po_num(3)=z;
                    if po_num(1) == 0 && po_num(2) == 0 && po_num(3) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), po_num(3), 0, [], []};                        
                        q11q12q21q22 = [];
                        % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
                        % We assume that the precision is 0.02mm (1) as industry manipulators
                        % Ci_in_Ob: Ci in frame Ob-xyz
                        C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
                        C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
                        A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
                        % %-----------------------------------------------%
                        if abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs((C2_in_Ob(1) - A2(1)) > 1e-2 || abs(C2_in_Ob(2) - A2(2)) > 1e-2)
                            Mode = 3; 
                            display( 'Mode switch to: "3T1R-SingularityA1C1"');
%                             set(handles.text12,'visible','on');
%                             set(handles.text12,'String','Mode switch to: "3T1R-SingularityA1C1"');
%                             set(handles.text10,'visible','on');
%                             set(handles.text10,'String','?11');
%                             set(handles.edit10,'visible','on');
                            po_cell_7 = inputdlg({'q11: SingularityA1C1'},'3T1R', [1 20]);
                           % po{7} = str2num(get(handles.edit10,'string')) * pi / 180;
                            po{7} = str2num(po_cell_7{1}) * pi / 180;
                            q11 = po{7};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                            obj3T1R = RCB3T1RSingularityA1C1(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
                        elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                            Mode = 4; 
                            display( 'Mode switch to: "3T1R-SingularityA2C2"');
%                             set(handles.text12,'visible','on');
%                             set(handles.text12,'String','Mode switch to: "3T1R-SingularityA2C2"');
%                             set(handles.text10,'visible','on');
%                             set(handles.text10,'String','?21');
%                             set(handles.edit10,'visible','on');
                            po_cell_8 = inputdlg({'q21: SingularityA2C2'},'3T1R-singularityA2C2', [1 20]);
                            %po{8} = str2num(get(handles.edit10,'String')) * pi / 180;
                            po{8} = str2num(po_cell_8{1}) * pi / 180;
                            q21 = po{8};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                            obj3T1R = RCB3T1RSingularityA2C2(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
                        elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                            Mode = 5; 
                            display( 'Mode switch to: "3T1R-SingularityA1C1A2C2"');
%                             set(handles.text12,'visible','on');
%                             set(handles.text12,'String','Mode switch to: "3T1R-SingularityA1C1A2C2"');
%                             set(handles.text10,'visible','on');
%                             set(handles.text10,'String','?11');
%                             set(handles.edit10,'visible','on');
%                             set(handles.text11,'String','?21');
%                             set(handles.edit11,'visible','on');
                            po_cell_z78 = inputdlg({'q11','q21'},'3T1R-singularityA1C1', [1 20;1 20]);
%                             po{7} = str2num(get(handles.edit10,'string')) * pi / 180;
%                             po{8} = str2num(get(handles.edit11,'string')) * pi / 180;
                            po{7} = str2num(po_cell_z78{1}) * pi / 180;
                            po{8} = str2num(po_cell_z78{2}) * pi / 180;
                            q11 = po{7};
                            q21 = po{8};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                            obj3T1R = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1A2C2_IK;
                        else
                            obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
                        end                        
                    end
                    
                    %%
                case 2 % 3T1R                    
                    %%------------------------ 3T1R IK ------------------------
                    % 3T1R mode:  [1 1 1 1 0 0]
                    % p = [x, y, z, alpha, [], []];
                    
                    po_num(1) = x; po_num(2) = y; po_num(3) = z; po_num(4) = theta * pi / 180;
                    %po_num = [77.781745930520227684092879831533, -32.2183, 150, -pi/4]; SingularityA1C1
                    %po_num = [77.781745930520227684092879831533, 32.2183,  150, pi/4]; SingularityA2C2
                    %po_num = [100, 100,  150, pi/4];
                    
                    if po_num(1) == 0 && po_num(2) == 0 && po_num(3) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), po_num(3), po_num(4), [], [], [], []};                        
                        q11q12q21q22 = [];
                        % Judge the sigularity as 1st+5th axes of C1A1 and C2A2 overlap
                        % We assume that the precision is 0.02mm (1) as industry manipulators
                        % Ci_in_Ob: Ci in frame Ob-xyz
                        C1_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0,-l1/2,0]')' + [po{1}, po{2}, po{3}];
                        C2_in_Ob = (eul2rotm([po{4}, 0, 0]) * [0, l1/2,0]')' + [po{1}, po{2}, po{3}];
                        A1 = (rotz(q0)*[0, -l1/2, 0]')'; A2 = (rotz(q0)*[0, l1/2, 0]')';
                        % %-----------------------------------------------%
                        if abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs((C2_in_Ob(1) - A2(1)) > 1e-2 || abs(C2_in_Ob(2) - A2(2)) > 1e-2)
                            Mode = 3; display( 'Mode switch to: "3T1R-SingularityA1C1"');
                            po_cell_7 = inputdlg({'q11: SingularityA1C1'},'3T1R', [1 20]);
                            po{7} = str2num(po_cell_7{1}) * pi / 180;
                            q11 = po{7};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, []};
                            obj3T1R = RCB3T1RSingularityA1C1(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
                        elseif abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2 && abs((C1_in_Ob(1) - A1(1)) > 1e-2 || abs(C1_in_Ob(2) - A1(2)) > 1e-2)
                            Mode = 4; display( 'Mode switch to: "3T1R-SingularityA2C2"');
                            po_cell_8 = inputdlg({'q21: SingularityA2C2'},'3T1R-singularityA2C2', [1 20]);
                            po{8} = str2num(po_cell_8{1}) * pi / 180;
                            q21 = po{8};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], [], q21};
                            obj3T1R = RCB3T1RSingularityA2C2(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
                        elseif abs(C1_in_Ob(1) - A1(1)) <= 1e-2 && abs(C1_in_Ob(2) - A1(2)) <= 1e-2 && abs(C2_in_Ob(1) - A2(1)) <= 1e-2 && abs(C2_in_Ob(2) - A2(2)) <= 1e-2
                            Mode = 5; display( 'Mode switch to: "3T1R-SingularityA1C1A2C2"');
                            po_cell_z78 = inputdlg({'q11','q21'},'3T1R-singularityA1C1', [1 20,1 20]);
                            po{7} = str2num(po_cell_z78{1}) * pi / 180;
                            po{8} = str2num(po_cell_z78{2}) * pi / 180;
                            q11 = po{7};
                            q21 = po{8};
                            PosOri = {po{1}, po{2}, po{3}, po{4}, [], [], q11, q21};
                            obj3T1R = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1A2C2_IK;
                        else
                            obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_IK;
                        end                        
                    end
                    
                    %%
                case 3 % 3T1R-SingularityA1C1                      
                    %po_num = [77.781745930520227684092879831533, -32.2183, 150, -pi/4]; SingularityA1C1
                    display( 'Mode switch to: "3T1R-SingularityA1C1"');
                    po_num(3) = z; 
                    po_num(4) = theta * pi / 180;
                    po_num(1) = -l1/2 * sin(po_num(4));
                    po_num(2) = l1/2 * (cos(po_num(4)) - 1);
                    q11 = q11 * pi / 180;
                    po = {po_num(1), po_num(2), po_num(3), po_num(4), [], [], q11, []};
                    q11q12q21q22 = [];
                    obj3T1R = RCB3T1RSingularityA1C1(po, q11q12q21q22, l1, l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1_IK;
                
                    %%
                case 4 % 3T1R-SingularityA2C2                    
                    %po_num = [77.781745930520227684092879831533, 32.2183,  150, pi/4]; SingularityA2C2
                    display( 'Mode switch to: "3T1R-SingularityA2C2"');
                    po_num(3) = z; 
                    po_num(4) = theta * pi / 180;
                    po_num(1) = l1/2 * sin(po_num(4));
                    po_num(2) = l1/2 * (- cos(po_num(4)) + 1);
                    q21 = q21 * pi / 180;
                    po = {po_num(1), po_num(2), po_num(3), po_num(4), [], [], [], q21};
                    q11q12q21q22 = [];
                    obj3T1R = RCB3T1RSingularityA2C2(po, q11q12q21q22, l1, l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA2C2_IK;
                  
                    
                    %%
                case 5 % 3T1R-SingularityA1C1A2C2
                    display( 'Mode switch to: "3T1R-SingularityA1C1A2C2"');
                    po{3} = z;
                    po{7} = q11 * pi / 180;
                    po{8} = q21 * pi / 180;
                    q11 = po{7};
                    q21 = po{8};
                    q11q12q21q22 = [];
                    PosOri = {0, 0, po{3}, 0, [], [], q11, q21};
                    po = PosOri;
                    obj3T1R = RCB3T1RSingularityA1C1A2C2(PosOri, q11q12q21q22, l1, l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj3T1R.RCB_3T1R_SingularityA1C1A2C2_IK;   
                 
                    %%
                case 6
                    %%------------------------ 2T2R IK: Six-bar ------------------------
                    % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
                    % p = [x, y, z, [], [], gamma]
                    po_num(1)=x;
                    po_num(2)=y;
                    po_num(3)=z;
                    po_num(4) = theta * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0  
                        if po_num(3) == 0
                            po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                        elseif po_num(4) ~= 0
                            % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
                            % p = [[], [], z, [], beta, gamma]; x = y = 0
                            Mode = 7;
                            po_cell = inputdlg({'q11 =(-360,360)'},'2T2R-6-Bar', [1 20]);
                            po_num(5) = str2num(po_cell{1}) * pi / 180;
                            po = {[], [], po_num(3), [], po_num(5), po_num(4)};
                            q11q12q14q23 = [];
                            obj1T2RRotAroundPoint = RCB1T2RRotAroundPoint(po,q11q12q14q23,l1,l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj1T2RRotAroundPoint.RCB_1T2R_RotAroundPoint_IK;
                        elseif po_num(4) == 0
                            %Mode = 8;
                            %po_num(5) = 90 * pi / 180;
                            po = {0, 0, po_num(3), [], [], po_num(4), pi/2, pi/2};
                            q11q12q14q23 = [];
                            obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
                            [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                        end
                    else
                        po = {po_num(1), po_num(2), po_num(3), [], [], po_num(4)};
                        q11q12q14q23 = [];
                        obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
                        [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                    end
                 
                    %%
                case 7
                    %%------------------------ 2T2R IK£ºRotate around point  ------------------------
                    % Mechanism rotate around point p(1:3):  [0 0 1 0 1 1]
                    % p = [[], [], z, [], beta, gamma]; x = y = 0
                    po_num(2)=q11 * pi / 180; 
                    po_num(3)=theta * pi / 180;
                    po_num(1) = z;
                    po = {[], [], po_num(1), [], po_num(2), po_num(3)};
                    q11q12q14q23 = [];
                    obj1T2RRotAroundPoint = RCB1T2RRotAroundPoint(po,q11q12q14q23,l1,l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj1T2RRotAroundPoint.RCB_1T2R_RotAroundPoint_IK;
                 
                    %%
                case 8
                    %%------------------------ 2T1R IK£ºPlanar Five-bar ------------------------
                    % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 1 1 1]
                    % p = [x, 0, z, [], beta, 0]
                    po_num(1)=x;
                    po_num(2)=z;
                    po_num(3) = theta * pi / 180;
                    po = {po_num(1), 0, po_num(2), [], po_num(3), []};
                    q11q12q14q22 = [];
                    obj2T2Rfivebar = RCB2T2Rfivebar(po,q11q12q14q22,l1,l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK;
                    
                    %%
                case 9
                    %%------------------------- 2T1R IK£ºPlanar Three-bar -----------------------
                    % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 1]
                    % p = [x, 0, z, 0, beta, []]
                    po_num(1)=x;
                    po_num(2)=z;
                    po_num(3) = theta * pi / 180;
                    po = {po_num(1), 0, po_num(2), 0, po_num(3), []};
                    q11q12q14q23 = [];
                    obj2T2Rthreebar = RCB2T2Rthreebar(po,q11q12q14q23,l1,l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2T2Rthreebar.RCB_2T2R_ThreeBar_IK;
                    
                    %%
                case 10
                    %-------------------------- Branch Chain A1C1 --------------------------------
                    % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                    % p = {x, y, [], [], [], []}; y < 0
                    q11q12q22q13 = [];
                    po_num(1)=x; 
                    po_num(2)=y;
                    po_num(3) = q12 * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0
                        po_cell = inputdlg({'theta11 =(-360~360)','theta12 =(0~45))','theta21 =(-360~360)','theta22 =(0~45))'},'Fixed-SerialA1C1A2C2', [1 20; 1 20; 1 20; 1 20]);
                        for i=1:4 po_num(i) = str2num(po_cell{i}) * pi / 180; end
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), [], [], [], [], po_num(3)};
                        obj2RserialA1C1 = RCB2RserialA1C1(po,q11q12q22q13,l1,l2);
                        [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
                    end
                 
                    %%
                case 11
                    %-------------------------- Branch Chain A2C2 --------------------------------
                    % Four-bar linkage with Serial Chain A1C1:  [1 1 0 0 0 0]
                    % p = {x, y, [], [], [], []}; y > 0
                    q11q12q22q23 = [];
                    po_num(1)=x;
                    po_num(2)=y;
                    po_num(3) = q22 * pi / 180;
                    if po_num(1) == 0 && po_num(2) == 0
                        po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};
                    else
                        po = {po_num(1), po_num(2), [], [], [], [], po_num(3)};
                        obj2RserialA2C2 = RCB2RserialA2C2(po,q11q12q22q23,l1,l2);
                        [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
                    end
                
                    %%
                case 12
                    %%------------------------- Two Serial Chains -----------------------------
                    % Mechanism transit into two serial chain mechanism:  [1 1 1 0 0 0]
                    % p = {0, 0, 0, [], [], []}
                    q11q12q21q22 = [];
                    po_num(1) = q11 * pi / 180;
                    po_num(2) = q12 * pi / 180;
                    po_num(3) = q21 * pi / 180;
                    po_num(4) = q22 * pi / 180;
                    po = {0, 0, 0, [], [], [], po_num(1), po_num(2), po_num(3), po_num(4)};                    
                    objRCBFixedSerialChain = RCBFixedSerialChain(po,q11q12q21q22,l1,l2);
                    [p, EulerAngle_q11_theta, ABC, q1q2, WSvalue] = objRCBFixedSerialChain.RCB_FixedSerialChain_IK;
            end