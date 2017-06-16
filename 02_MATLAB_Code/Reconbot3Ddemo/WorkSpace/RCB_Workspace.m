

%%-------------------------------------------------------------------------
%%---------------2-RER Workspace selection and Generation -----------------
%%-------------------------------------------------------------------------

%
%     This is written by Wan Ding in 15 Feb 2017.
%     The copyright is belong to Dr. Wan Ding.

%       po = {cell}               Position and rotation of the MP, cell
%       po_cell = {cell}          Position and rotation of the MP, cell
%       po_num = {num}            Position and rotation of the MP, number
%       SolutionRow               The row of solution in 8 solutions.
%       EulerAngle_q11_theta      Euler Angle(Z-Y-X)  q11, and theta: [alpha, beta, gamma, q11, theta]

%%-------------------------------------------------------------------
clc
clear
%clf

%% Initialization
l1 = 220;
l2 = 147.25;
deg = pi/180;

% Enalbe all folders inside "SSoop"
addpath(genpath(pwd)); 
% 3D demo
%InitHome;   

%% Choose the configuration of each trajectory point
addpath(genpath(pwd)); % Enalbe folder "RCB_3D_demo"
str = {'3T1R';'2T2R-6-Bar';'2T2R-5-Bar';'2T1R-3-BarSerial';'2R-SerialA1C1';...
    '2R-SerialA2C2';'Fixed-SerialA1C1A2C2';'SinguraitA1C1';'SinguraitA2C2';};
[s,v] = listdlg('PromptString','Select a Operation Mode:','SelectionMode','single',...
    'ListString',str);
if v == 0
    %break;
end

%% Workspace geneation of Selected Modes
switch s
    % input: length, width, height and step of the Study Points
    case 1
        % ---- 3T1R ------
        x_bound = [-320, 320];y_bound = [-320, 320];z_bound = [-10, 320];
        StepLength = 10;         
    case 2
        % ---- 2T2R ------
        x_bound = [-320, 320];y_bound = [-320, 320];z_bound = [-10, 320];
        StepLength = 10;
    case 3
        % ---- Five-Bar ------
        x_bound = [-300, 300];y_bound = [0, 0];z_bound = [0, 300];        
        StepLength = 5;
    case 4
        % ---- Three-Bar ------
        x_bound = [-300, 300];y_bound = [0, 0];z_bound = [0, 300];        
        StepLength = 5;
    case 5
        % ---- 2RserialA1C1 ------
        % angle1 is an angle project on plane xoy
        % angle2 is an angle with plane xoy
        angle1 = [-2*pi/3, 2*pi/3]; angle2 = [0, pi/2];
        stepAngle = 200;
    case 6
        % ---- 2RserialA2C2 ------
        % angle1 is an angle project on plane xoy
        % angle2 is an angle with plane xoy
        angle1 = [-2*pi/3, 2*pi/3]; angle2 = [0, pi/2];
        stepAngle = 200;
    case 8
        % ---- 3T1R SingularityPositionA1C1 ------
        alpha = [-2*pi/3, 2*pi/3];z_bound = [0, 320];        
        stepAlpha = 200;
        stepZ = 100;
    case 9
        % ---- 3T1R SingularityPositionA2C2 ------
        alpha = [-2*pi/3, 2*pi/3];z_bound = [0, 320];        
        stepAlpha = 30;
        stepZ = 10;
end

%% ----------------- Original Point Calculation  --------------
BaseLow_CenterPointBearing = [250, 250, 88.97];
BaseUP_CenterPointBearing = [170, 160, 5.47];
BaseUP_CenterPointA1C1 = [170, 50, 58.88];
BaseJointA1C1A2C2_CenterPointSidePlate = [0, 48.10, 28];

OriginalPoint(1) = BaseLow_CenterPointBearing(1);
OriginalPoint(2) = BaseLow_CenterPointBearing(2);
OriginalPoint(3) = BaseLow_CenterPointBearing(3) + BaseUP_CenterPointA1C1(3) + BaseJointA1C1A2C2_CenterPointSidePlate(3) - BaseUP_CenterPointBearing(3);

% plot3(OriginalPoint(1), OriginalPoint(2), OriginalPoint(3),'r.','MarkerSize',10); 
TransCuboidPath2OriginalPoint = [[eye(3,3);0,0,0], [OriginalPoint, 1]'];


%% ----------------- Calculation Path Geneation  --------------
% Because there are spatial body and surface motion;
% S<=2 is spatial body motion, the workspace calculation range should be a cube;
% S > 2 is spatial and plane surface motion, the workspace calculation range should be a surface;
if s <= 4
    % ---- Cuboid Path Geneation Test ----
    [CuboidPath, xyzStepLength] = pathGeneration(x_bound,y_bound,z_bound,StepLength);
    %plot3(CuboidPath(:,3), CuboidPath(:,4), CuboidPath(:,5),'r.','MarkerSize',1); axis equal
    save('pathCuboid.mat','CuboidPath');
    % ----------------- plot Path output in 3Ddemo original point  --------------
    pathOP = [CuboidPath(:,3:5),ones(length(CuboidPath(:,1)),1)] * TransCuboidPath2OriginalPoint'; % pathOP: path output
    sizepathOP = size(pathOP);
    i = 1:sizepathOP;
    % plot3(pathOP(i,1), pathOP(i,2), pathOP(i,3),'r.','MarkerSize',1);
    % hold on;
    axis equal;
elseif s == 5 || s == 6 
    % angle1 is an angle project on plane xoy
    % angle2 is an angle with plane xoy
    for i_angle2 = 1:1:stepAngle
        angle2Intepetation(i_angle2) = angle2(1) + i_angle2 * (angle2(2) - angle2(1)) / stepAngle;
        zIntepetation(i_angle2) = l1 * sin(angle2Intepetation(i_angle2)) / 2;
        for i_angle1 = 1:1:stepAngle            
            angle1Intepetation(i_angle1) = angle1(1) + i_angle1 * (angle1(2) - angle1(1)) / stepAngle;
            if s == 5 % --- A1C1 ---
                xIntepetation(i_angle1) = - l1 * cos(angle2Intepetation(i_angle2)) * sin(angle1Intepetation(i_angle1)) / 2;
                yIntepetation(i_angle1) = l1 * (cos(angle2Intepetation(i_angle2)) * cos(angle1Intepetation(i_angle1)) - 1) / 2;                
            elseif s == 6 % --- A2C2 ---
                xIntepetation(i_angle1) = l1 * cos(angle2Intepetation(i_angle2)) * sin(angle1Intepetation(i_angle1)) / 2;
                yIntepetation(i_angle1) = l1 * (1 - cos(angle2Intepetation(i_angle2)) * cos(angle1Intepetation(i_angle1))) / 2;
            end
        end
        SpherePath((1 + stepAngle * (i_angle2 -1)):i_angle2  * stepAngle,:) = [xIntepetation',yIntepetation',zIntepetation(i_angle2)*ones(stepAngle,1)];
    end
    % ----------------- plot Path output in 3Ddemo original point  --------------
    pathOP = [SpherePath(:,1:3),ones(length(SpherePath(:,1)),1)] * TransCuboidPath2OriginalPoint'; % pathOP: path output
    sizepathOP = size(pathOP);
    xyzStepLength = [1, stepAngle, stepAngle];
    %plot3(pathOP(:,1), pathOP(:,2), pathOP(:,3),'r.','MarkerSize',1);
    %axis equal    
else
    % ---- Cycle Path Generation for Singularity Positions AiCi ----
    for i_alpha = 1:1:stepAlpha
        alphaIntepetation(i_alpha) = alpha(1) + i_alpha * (alpha(2) - alpha(1)) / stepAlpha;        
        if s == 8
            xIntepetation(i_alpha) = - l1 * sin(alphaIntepetation(i_alpha)) / 2;
            yIntepetation(i_alpha) = l1 * (cos(alphaIntepetation(i_alpha)) - 1) / 2;
        elseif s == 9
            xIntepetation(i_alpha) = l1 * sin(alphaIntepetation(i_alpha)) / 2;
            yIntepetation(i_alpha) = l1 * (1 - cos(alphaIntepetation(i_alpha)) ) / 2;
        end
    end
    x_bound = [min(xIntepetation),max(xIntepetation)];
    y_bound = [min(yIntepetation),max(yIntepetation)];
    for i_z = 1:1:stepZ
        zIntepetation(i_z) = z_bound(1) + i_z * (z_bound(2) - z_bound(1)) / stepZ;
    end
    for i_Cylinder = 1:1:stepZ
        CylinderPath((1 + stepAlpha * (i_Cylinder-1)):i_Cylinder * stepAlpha,:) = [xIntepetation',yIntepetation',zIntepetation(i_Cylinder)*ones(stepAlpha,1)];
    end
    % ----------------- plot Path output in 3Ddemo original point  --------------
    pathOP = [CylinderPath(:,1:3),ones(length(CylinderPath(:,1)),1)] * TransCuboidPath2OriginalPoint'; % pathOP: path output
    sizepathOP = size(pathOP);
    xyzStepLength = [1, stepAlpha, stepZ];
    %plot3(pathOP(:,1), pathOP(:,2), pathOP(:,3),'r.','MarkerSize',1);
    %axis equal
end

%Workspace_xyzbound = [x_bound;y_bound;z_bound];
%-----------------------------------------------------------

%% Judge the exsitence of each point
WSvalue = [];
step = 0;
NumExistedPoint = 0;
tic
for k = 1:1:xyzStepLength(3)
    for j = 1:1:xyzStepLength(2)
        for i = 1:1:xyzStepLength(1)            
            step = step + 1;
            switch s
                case 1
                     % 3T1R mode:  [1 1 1 1 0 0]
                     % p = [x, y, z, alpha, [], []];
                    po = {CuboidPath(step,3), CuboidPath(step,4), CuboidPath(step,5), 0, [], [], 0, 0};
                    q11q12q21q22 = [];
                    obj3T1R = RCB3T1R(po, q11q12q21q22, l1, l2);
                    [~, ~, ~, ~, WSvalue_3T1R] = obj3T1R.RCB_3T1R_IK;    
                    WSvalue(i,j,k) = WSvalue_3T1R(1);
                case 2
                    % Mechanism in a general six-bar linkage:  [1 1 1 0 0 1]
                    % p = [x, y, z, [], [], gamma]
                    po = {CuboidPath(step,3), CuboidPath(step,4), CuboidPath(step,5), [], [], 0};
                    q11q12q14q23 = [];
                    obj2T2Rsixbar = RCB2T2Rsixbar(po,q11q12q14q23,l1,l2);
                    WSvalue(i,j,k) = obj2T2Rsixbar.RCB_2T2Rsixbar_IK;
                case 3 
                    % Mechanism transfers into Planar five-bar Linkage:  [1 1 1 0 1 1]
                    % p = [x, 0, z, [], beta, 0]                    
                    po = {CuboidPath(step,3), 0, CuboidPath(step,5), [], 0, 0}; % Angle should be rad: 'pi/3;
                    q11q12q14q22 = [];
                    obj2T2Rfivebar = RCB2T2Rfivebar(po,q11q12q14q22,l1,l2);
                    WSvalue(i,j,k) = obj2T2Rfivebar.RCB_2T2R_FiveBar_IK; 
                case 4
                    % Mechanism transfers into Planar three-bar Linkage:  [1 1 1 1 1 0]
                    % p = [x, 0, z, 0, beta, []]
                    po = {CuboidPath(step,3), 0, CuboidPath(step,5), 0, 0, []}; % Angle should be rad: 'pi/3;
                    q11q12q14q23 = [];
                    obj2T2Rthreebar = RCB2T2Rthreebar(po,q11q12q14q23,l1,l2);
                    WSvalue(i,j,k) = obj2T2Rthreebar.RCB_ThreeBar_IK; 
                case 5
                    % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 1 0 0 0]
                    % p = [x, y, z, [], [], []]; y < 0 
                    % Here we send 'z' coordinate to make sure the
                    % calculated zop = z, otherwise, it will be wrong value
                    po = {SpherePath(step,1), SpherePath(step,2), SpherePath(step,3), [], [], [], pi};
                    q11q12q22q13 = [];
                    obj2RserialA1C1 = RCB2RserialA1C1(po,q11q12q22q13,l1,l2);
                    WSvalue(i,j,k) = obj2RserialA1C1.RCB_2R_SerialA1C1_IK;
                case 6
                    % Four-bar linkage with Serial Chain A1C1: ----- isempty(p) = [1 1 1 0 0 0]
                    % p = [x, y, z, [], [], []]; y > 0 
                    % Here we send 'z' coordinate to make sure the
                    % calculated zop = z, otherwise, it will be wrong value
                    po = {SpherePath(step,1), SpherePath(step,2), SpherePath(step,3), [], [], [], pi};
                    q11q12q22q23 = [];
                    obj2RserialA2C2 = RCB2RserialA2C2(po,q11q12q22q23,l1,l2);
                    WSvalue(i,j,k) = obj2RserialA2C2.RCB_2R_SerialA2C2_IK;
                 case 8
                     % 3T1R mode-SingularityPositionA1C1:  [1 1 1 1 0 0]
                     % p = [x, y, z, alpha, [], []];                     
                    po = {CylinderPath(step,1), CylinderPath(step,2), CylinderPath(step,3), alphaIntepetation(j), [], []};
                    q11q12q21q22 = [];
                    obj3T1R_SinguPosAiCi = RCB3T1R_SinguPosAiCi(po, q11q12q21q22, l1, l2);
                    [WSvalue(i,j,k),~] = obj3T1R_SinguPosAiCi.RCB_3T1R_SinguPosAiCi_IK; 
                  case 9
                     % 3T1R mode-SingularityPositionA2C2:  [1 1 1 1 0 0]
                     % p = [x, y, z, alpha, [], []];
                    po = {CylinderPath(step,1), CylinderPath(step,2), CylinderPath(step,3), alphaIntepetation(j), [], []};
                    q11q12q21q22 = [];
                    obj3T1R_SinguPosAiCi = RCB3T1R_SinguPosAiCi(po, q11q12q21q22, l1, l2);
                    [~, WSvalue(i,j,k)] = obj3T1R_SinguPosAiCi.RCB_3T1R_SinguPosAiCi_IK;  
            end
            if WSvalue(i,j,k) == 1
                NumExistedPoint = NumExistedPoint + 1;
                %WS_po_value(NumExistedPoint, :) = CuboidPath(step,3:5);
                WS_po_value(NumExistedPoint, :) = pathOP(step,1:3);
            end
        end
    end
end
toc

save('WS_po_value.mat','WS_po_value');

%% Get the boundary of the Workspace
k = boundary(WS_po_value,1);

if s == 3 || s == 4 || s == 5 || s == 6 || s == 8 || s == 9
    plot3(WS_po_value(:,1), WS_po_value(:,2), WS_po_value(:,3),'b.','MarkerSize',5);
else
    %trisurf(k,WS_po_value(:,1), WS_po_value(:,2), WS_po_value(:,3),'Facecolor','yellow','FaceAlpha',0.5,'LineStyle', '-')
    trisurf(k,WS_po_value(:,1), WS_po_value(:,2), WS_po_value(:,3),'Facecolor','red','FaceAlpha',0.5,'LineStyle', 'none')
end
axis equal
grid on
hold on
%% Show the Workspace

% save('q0q1q2_mat.mat')
% %% 3D Animation
% for i = 1:length(q0q1q2_mat)
%     %========================== Animation ============================
%     ReconbotANI(q0q1q2_mat(i,:));
%     set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-', 'LineWidth',2); hold off
%     %============================ End ================================
% end