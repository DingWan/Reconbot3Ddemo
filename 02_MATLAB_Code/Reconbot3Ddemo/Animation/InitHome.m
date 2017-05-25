function InitHome
% Use forward kinematics to place the robot in a specified
% configuration.
% Figure setup data, create a new figure for the GUI

% Loads all the link data from file linksdata.mat.
% This data comes from a Pro/E 3D CAD model and was made with cad2matdemo.m
% from the file exchange.  All link data manually stored in linksdata.mat
[linkdata]=load('RCBLinkdata.mat');
%,'BaseLow','BaseUP', 'BaseJointA1C1','LowLinkA1C1',...
%'UpLinkA1C1','UPjointA1C1','BaseJointA2C2','LowLinkA2C2','UpLinkA2C2','UPjointA2C2','MovingPlatform'

%Place the robot link 'data' in a storage area
setappdata(0,'Link_BaseLow_data',linkdata.BaseLow);
setappdata(0,'Link_BaseUP_data',linkdata.BaseUP);
setappdata(0,'Link_BaseJointA1C1_data',linkdata.BaseJointA1C1);
setappdata(0,'Link_LowLinkA1C1_data',linkdata.LowLinkA1C1);
setappdata(0,'Link_UpLinkA1C1_data',linkdata.UpLinkA1C1);
setappdata(0,'Link_UPjointA1C1_data',linkdata.UPjointA1C1);
setappdata(0,'Link_BaseJointA2C2_data',linkdata.BaseJointA2C2);
setappdata(0,'Link_LowLinkA2C2_data',linkdata.LowLinkA2C2);
setappdata(0,'Link_UpLinkA2C2_data',linkdata.UpLinkA2C2);
setappdata(0,'Link_UPjointA2C2_data',linkdata.UPjointA2C2);
setappdata(0,'Link_MovingPlatform_data',linkdata.MovingPlatform);


set(0,'Units','pixels')
dim = get(0,'ScreenSize');
fig_1 = figure('doublebuffer','on','Position',[0,35,dim(3)-200,dim(4)-110],...
    'Name',' Reconbot Graphical Demo',...
    'NumberTitle','off','CloseRequestFcn',@del_app);
hold on;
light('Position',[-100 -100 700]);
daspect([1 1 1])                    % Setting the aspect ratio
view(45,25)
xlabel('X'),ylabel('Y'),zlabel('Z');
title('IGM Robotics - Reconbot');
axis([-100 600 -100 600 0 700]);
grid on;

BaseLow_data = getappdata(0,'Link_BaseLow_data');
BaseUP_data = getappdata(0,'Link_BaseUP_data');
BaseJointA1C1_data = getappdata(0,'Link_BaseJointA1C1_data');
LowLinkA1C1_data = getappdata(0,'Link_LowLinkA1C1_data');
UpLinkA1C1_data = getappdata(0,'Link_UpLinkA1C1_data');
UPjointA1C1_data = getappdata(0,'Link_UPjointA1C1_data');
BaseJointA2C2_data = getappdata(0,'Link_BaseJointA2C2_data');
LowLinkA2C2_data = getappdata(0,'Link_LowLinkA2C2_data');
UpLinkA2C2_data = getappdata(0,'Link_UpLinkA2C2_data');
UPjointA2C2_data = getappdata(0,'Link_UPjointA2C2_data');
MovingPlatform_data = getappdata(0,'Link_MovingPlatform_data');
%

%The 'home' position, for init.
q0 = 0;        q11 = 0;        q12 = 45;        q13 = 90;        q14 = -45;        q15 = 0;
q21 = 0;        q22 = 45;       q23 = 90;        q24 = -45;        q25 = 0;
%q0q1q2_HomePosition = [0, -pi, pi/4, pi/2, -pi/4, -pi, 0, pi/4, pi/2, -pi/4, 0];

q0q1q2 = [q0, q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
[T_01,T_1_02,T_1_03,T_1_04,T_1_05,T_1_06,T_2_02,T_2_03,T_2_04,T_2_05] = RCB_3Ddemo_ForwardKinematics(q0q1q2);

% We can display the collision lines of each components
% [~, ~, ~] = CollisionPointsFK(q0q1q2);

% Actual vertex data of robot links
Link_BaseLow        =           BaseLow_data.V1;
Link_BaseUP         = (T_01   * BaseUP_data.V2')';
Link_BaseJointA1C1  = (T_1_02 * BaseJointA1C1_data.V3')';
Link_LowLinkA1C1    = (T_1_03 * LowLinkA1C1_data.V4')';
Link_UpLinkA1C1     = (T_1_04 * UpLinkA1C1_data.V5')';
Link_UPjointA1C1    = (T_1_05 * UPjointA1C1_data.V6')';

Link_MovingPlatform = (T_1_06 * MovingPlatform_data.V11')';

Link_BaseJointA2C2  = (T_2_02 * BaseJointA2C2_data.V7')';
Link_LowLinkA2C2    = (T_2_03 * LowLinkA2C2_data.V8')';
Link_UpLinkA2C2     = (T_2_04 * UpLinkA2C2_data.V9')';
Link_UPjointA2C2    = (T_2_05 * UPjointA2C2_data.V10')';

% points are no fun to watch, make it look 3d.
L1 = patch('faces', BaseLow_data.F1, 'vertices' ,Link_BaseLow(:,1:3));
L2 = patch('faces', BaseUP_data.F2, 'vertices' ,Link_BaseUP(:,1:3));
L3 = patch('faces', BaseJointA1C1_data.F3, 'vertices' ,Link_BaseJointA1C1(:,1:3));
L4 = patch('faces', LowLinkA1C1_data.F4, 'vertices' ,Link_LowLinkA1C1(:,1:3));
L5 = patch('faces', UpLinkA1C1_data.F5, 'vertices' ,Link_UpLinkA1C1(:,1:3));
L6 = patch('faces', UPjointA1C1_data.F6, 'vertices' ,Link_UPjointA1C1(:,1:3));

L11 = patch('faces', MovingPlatform_data.F11, 'vertices' ,Link_MovingPlatform(:,1:3));

L7 = patch('faces', BaseJointA2C2_data.F7, 'vertices' ,Link_BaseJointA2C2(:,1:3));
L8 = patch('faces', LowLinkA2C2_data.F8, 'vertices' ,Link_LowLinkA2C2(:,1:3));
L9 = patch('faces', UpLinkA2C2_data.F9, 'vertices' ,Link_UpLinkA2C2(:,1:3));
L10 = patch('faces', UPjointA2C2_data.F10, 'vertices' ,Link_UPjointA2C2(:,1:3));

Tr = plot3(0,0,0,'b.'); % holder for trail paths
CPsA1C1 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A1C1
CPsA2C2 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A2C2
%
setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,Tr,CPsA1C1,CPsA2C2])
%
setappdata(0,'xCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
setappdata(0,'yCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
setappdata(0,'zCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
%
setappdata(0,'xCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
setappdata(0,'yCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
setappdata(0,'zCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
%
setappdata(0,'xtrail',0); % used for trail tracking.
setappdata(0,'ytrail',0); % used for trail tracking.
setappdata(0,'ztrail',0); % used for trail tracking.
%
set(L1, 'facec', [105 105 105]/255);%105 105 105
set(L1, 'EdgeColor','none');
set(L2, 'facec', [250 235 215]/255);
set(L2, 'EdgeColor','none');
set(L3, 'facec', [105 105 105]/255);
set(L3, 'EdgeColor','none');
set(L4, 'facec', [255 182 193]/255);
set(L4, 'EdgeColor','none');
set(L5, 'facec', [135 206 250]/255);
set(L5, 'EdgeColor','none');
set(L6, 'facec', [105 105 105]/255);
set(L6, 'EdgeColor','none');

set(L11, 'facec', [250 235 215]/255);
set(L11, 'EdgeColor','none');

set(L7, 'facec', [105 105 105]/255);
set(L7, 'EdgeColor','none');
set(L8, 'facec', [255 182 193]/255);
set(L8, 'EdgeColor','none');
set(L9, 'facec', [135 206 250]/255);
set(L9, 'EdgeColor','none');
set(L10, 'facec', [105 105 105]/255);
set(L10, 'EdgeColor','none');
%
setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
%

end

function del_app(varargin)

clf
close
% delete;
end
 