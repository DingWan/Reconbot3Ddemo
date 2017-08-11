


function varargout = Reconbot(varargin)
% RECONBOT MATLAB code for Reconbot.fig

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Reconbot_OpeningFcn, ...
                   'gui_OutputFcn',  @Reconbot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
function Reconbot_OpeningFcn(hObject, eventdata, handles, varargin)

addpath(genpath(pwd));
get(0,'screensize');
handles.output = hObject;
guidata(hObject, handles);
axes(handles.axes5);
imshow(imread('D:\FreeAgntGoflex\RWTH-²©Ê¿ºó\Reconbot\Reconbot3Ddemo\02_MATLAB_Code\Reconbot3Ddemo\GUI\IGM.png'));
function varargout = Reconbot_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
function figure1_CloseRequestFcn(hObject, eventdata, handles)

delete(hObject);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%////////////Initialization/////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-------------------------------------CreateFcn of Axes-----------------------------------------% 

function axes1_CreateFcn(hObject, eventdata, handles)
[linkdata]=load('RCBLinkdata.mat');

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

light('Position',[-100 -100 700]);
daspect([1 1 1]) % Setting the aspect ratio
view(45,25)
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
%%
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

hold on;
%%
Tr = plot3(0,0,0,'b.'); % holder for trail paths

setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,Tr]);
% %
setappdata(0,'xtrail',0); % used for trail tracking.
setappdata(0,'ytrail',0); % used for trail tracking.
setappdata(0,'ztrail',0); % used for trail tracking.
% %
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

setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
function axes5_CreateFcn(hObject, eventdata, handles)



%--------------------------------------CreateFcn of Slider---------------------------------------% 

function slider1_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider2_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider3_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider4_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider5_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider6_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider7_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider8_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider9_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider10_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider11_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider12_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



%----------------------------------------CreateFcn of Edit--------------------------------------------% 

function edit1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit5_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit6_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit7_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit8_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_Traj_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit10_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit13_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit16_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit_RCB_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function edit35_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%----------------------------------------CreateFcn of popupmenue--------------------------------------% 

function popupmenu1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function popupmenu2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function popupmenu3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function popupmenu4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function text47_CreateFcn(hObject, eventdata, handles)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%////////////CallbackFunction/////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-----------------------------------------CallbackFcn of Slider----------------------------------------%

% slider 1 controls the step of 3D animation
function slider1_Callback(hObject, eventdata, handles)

Global_Para;
step_0=get(handles.slider1,'value');
n=length(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat);
step=round(step_0*n);
set(handles.edit35,'string',num2str(step));
ReconbotANI(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(step,:));

% slider 2-12 are input of x,y,z,q0,q1,q2
function slider2_Callback(hObject, eventdata, handles)
global x;
a=get(handles.slider2,'value');
x=a*250-147;
set(handles.edit1,'string',num2str(x));

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function slider3_Callback(hObject, eventdata, handles)
global y;
a=get(handles.slider3,'value');
y=a*250-147;
set(handles.edit2,'string',num2str(y));

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function slider4_Callback(hObject, eventdata, handles)
global z;
a=get(handles.slider4,'value');
z=a*250;
set(handles.edit3,'string',num2str(z));

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function slider5_Callback(hObject, eventdata, handles)
global theta;
a=get(handles.slider5,'value');
theta=a*180-90;
set(handles.edit4,'string',num2str(theta));

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function slider6_Callback(hObject, eventdata, handles)
global q0;
a=get(handles.slider6,'value');
q0=a*360-180;
set(handles.edit10,'string',num2str(q0));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider7_Callback(hObject, eventdata, handles)
global q11;
a=get(handles.slider7,'value');
q11=a*720-360;
set(handles.edit5,'string',num2str(q11));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider8_Callback(hObject, eventdata, handles)
global q12;
a=get(handles.slider8,'value');
q12=a*45;
set(handles.edit6,'string',num2str(q12));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider9_Callback(hObject, eventdata, handles)
global q14
a=get(handles.slider9,'value');
q14=a*90;
set(handles.edit13,'string',num2str(q14));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider10_Callback(hObject, eventdata, handles)
global q21;
a=get(handles.slider10,'value');
q21=a*720-360;
set(handles.edit7,'string',num2str(q21));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider11_Callback(hObject, eventdata, handles)
global q22;
a=get(handles.slider11,'value');
q22=a*45;
set(handles.edit8,'string',num2str(q22));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function slider12_Callback(hObject, eventdata, handles)
global q23;
a=get(handles.slider12,'value');
q23=a*90;
set(handles.edit16,'string',num2str(q23));

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end



%-----------------------------------------CallbackFcn of Pushbutton---------------------------------------%

%random position to home position
function pushbutton1_Callback(hObject, eventdata, handles)
RandomPosition2HomePosition;

%Enable the choose for IK or FK
function pushbutton2_Callback(hObject, eventdata, handles)
set(handles.popupmenu3,'enable','on');

%click 'NEXT' to make all the edit enable for the next point input
function pushbutton3_Callback(hObject, eventdata, handles)

Global_Para;

if choice==1
    
  Global_Para;
  SelectNumberOfTrajectoryPoints_GUI;

  for i=1:8
      EditList{i}=['edit',num2str(i)];
      TextList{i}=['text',num2str(i)];
  end

  for i=1:8
      set(handles.(EditList{i}),'enable','on');
      set(handles.(EditList{i}),'string','');
      set(handles.(TextList{i}),'enable','on');
  end
  
  set(handles.text1,'string','X');
  set(handles.text2,'string','Y');
  set(handles.text3,'string','Z');
  set(handles.text4,'string','?');
  set(handles.text5,'string','?11');
  set(handles.text6,'string','?12');
  set(handles.text7,'string','?21');
  set(handles.text8,'string','?22');
  
  set(handles.slider2,'enable','on','value',0);
  set(handles.slider3,'enable','on','value',0);
  set(handles.slider4,'enable','on','value',0);
  set(handles.slider5,'enable','on','value',0);
  set(handles.slider7,'enable','on','value',0);
  set(handles.slider8,'enable','on','value',0);
  set(handles.slider10,'enable','on','value',0);
  set(handles.slider11,'enable','on','value',0);
  
  
else
    
    set(handles.edit5,'enable','on');
    set(handles.edit7,'enable','on');
    set(handles.edit8,'enable','on');
    set(handles.edit10,'enable','on');
    set(handles.edit6,'enable','on');
    set(handles.edit13,'enable','on');
    set(handles.edit16,'enable','on');
    
    set(handles.text5,'enable','on');
    set(handles.text7,'enable','on');
    set(handles.text8,'enable','on');
    set(handles.text10,'enable','on');
    set(handles.text6,'enable','on');
    set(handles.text13,'enable','on');
    set(handles.text16,'enable','on');
    
   for i=2:12
       SliderList{i}=['slider',num2str(i)];
   end

   for i=2:12
       set(handles.(SliderList{i}),'enable','on');
       set(handles.(SliderList{i}),'value','0');
   end
  
   set(handles.slider6,'enable','off');
   set(handles.slider9,'enable','off');
   set(handles.slider12,'enable','off');
   
end
    
%click 'OK' to finish the input of one singel point of IK_ModeSelection
function pushbutton4_Callback(hObject, eventdata, handles)

Global_Para;

if choice==1
   
   Global_Para;
   workspace_error;
   hwait=waitbar(1,'Complete');
   pause(0.5);
   close(hwait);

   if isreal(q1q2) == 0 || (WSvalue(1) == 0 && WSvalue(2) == 0 && WSvalue(3) == 0)
      set(handles.text20,'visible','off');
      set(handles.edit_RCB,'visible','off');
      set(handles.pushbutton7,'visible','off');
   else
      set(handles.text20,'visible','on');
      set(handles.edit_RCB,'visible','on');
      set(handles.pushbutton7,'visible','on');
   end
   
else
    axes(handles.axes1);
    FK_Simulation;
    
end

%Get the result of original paln
function pushbutton5_Callback(hObject, eventdata, handles)

Global_Para;
Simulation;

hwait=waitbar(1,'Complete');
pause(0.5);
close(hwait);
    
%Clear all the axeses
function pushbutton6_Callback(hObject, eventdata, handles)

Global_Para;

cla(handles.axes2);
cla(handles.axes3);
cla(handles.axes4);

for i=1:8
    EditList{i}=['edit',num2str(i)];
    TextList{i}=['text',num2str(i)];
end

for i=1:8
    set(handles.(EditList{i}),'enable','on');
    set(handles.(EditList{i}),'string','');
    set(handles.(TextList{i}),'enable','on');
end

    set(handles.edit10,'enable','on');
    set(handles.edit10,'string','');

    set(handles.edit13,'enable','on');
    set(handles.edit13,'string','');
    set(handles.edit16,'enable','on');
    set(handles.edit16,'string','');
    
    set(handles.text10,'enable','on');
    set(handles.text13,'enable','on');
    set(handles.text16,'enable','on');
    
   for i=1:12
       SliderList{i}=['slider',num2str(i)];
   end

   for i=1:12
       set(handles.(SliderList{i}),'enable','on');
       set(handles.(SliderList{i}),'value',0);
   end
   

set(handles.text1,'string','X');
set(handles.text2,'string','Y');
set(handles.text3,'string','Z');
set(handles.text4,'string','?');
set(handles.text5,'string','?11');
set(handles.text6,'string','?12');
set(handles.text7,'string','?21');
set(handles.text8,'string','?22');
set(handles.text20,'visible','off');
set(handles.edit_RCB,'visible','off');
set(handles.pushbutton7,'visible','off');
set(handles.edit_RCB,'string','');
set(handles.pushbutton17,'visible','off');
set(handles.pushbutton4,'string','OK');
set(handles.edit35,'string','');
set(handles.text47,'string','Input the Num of Step')

ReconbotANI(q0q1q2_HomePosition);

IK_FinalPos=0;
FK_FinalPos=0;

set(Tr,'xdata',0,'ydata',0,'zdata',0);

%Click 'OK' to show the position oritation of the mechanismus 
function pushbutton7_Callback(hObject, eventdata, handles)

Global_Para;
IK_FinalPos=1;

IK_ModeSelection_GUI;

SolutionRow=str2num(get(handles.edit_RCB,'string'));

if SolutionRow >= 1 && SolutionRow <= length(q1q2(:,1))
   q0q1q2_display = [0, q1q2(SolutionRow,:)];
   q0q1q2_all = [zeros(length(q1q2(:,1)),1), q1q2];
   ReconbotANI(q0q1q2_display);
 
else

    errordlg('index exceeds matrix dimensions, Please Modify!','Input Error');
    
end

%Save the Output Data%
function pushbutton8_Callback(hObject, eventdata, handles)

Global_Para;

[filename,pathname]=uiputfile({'*.mat','Mat-File(*.mat)'},'Save Data');
str=strcat(pathname,filename);
save(char(str),'HomePos2SelectedEndPos_OutputData_Origin','HomePos2SelectedEndPos_OutputData_RePlan');

%Load the saved Data%
function pushbutton9_Callback(hObject, eventdata, handles)

[filename,pathname]=uigetfile({'*.mat','Mat-File(*.mat)'},'Load Data');
str=strcat(pathname,filename);
load(char(str));
assignin('base','HomePos2SelectedEndPos_OutputData_Origin',HomePos2SelectedEndPos_OutputData_Origin);
assignin('base','HomePos2SelectedEndPos_OutputData_RePlan',HomePos2SelectedEndPos_OutputData_RePlan);

%Re-Simulation for origin after Load%
function pushbutton10_Callback(hObject, eventdata, handles)

Global_Para;
Re_Simulation_Origin;

%Re-Simulation for RePlan after Load%
function pushbutton12_Callback(hObject, eventdata, handles)
 
Global_Para;
Re_Simulation_RePlan;

%Get the result of RePlan%
function pushbutton13_Callback(hObject, eventdata, handles)
 RePlan;
 
hwait=waitbar(1,'Complete');
pause(0.5);
close(hwait);

%Show the trail of the mechanismus 
function pushbutton14_Callback(hObject, eventdata, handles)

Global_Para;
axes(handles.axes1);
CenterPoint_MP;

function pushbutton17_Callback(hObject, eventdata, handles)

Global_Para;
FK_Finalpos=1;
axes(handles.axes1);
FK_FinalPosition;


%-------------------------------------------CallbackFcn of Edit---------------------------------------------%

% Input parameter of IK
function edit1_Callback(hObject, eventdata, handles)

global x;
x=str2num(get(handles.edit1,'string'));
set(handles.slider2,'value',(x+147)/250);

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function edit2_Callback(hObject, eventdata, handles)

global y;
y=str2num(get(handles.edit2,'string'));
set(handles.slider3,'value',(y+147)/250);

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function edit3_Callback(hObject, eventdata, handles)

global z;
z=str2num(get(handles.edit3,'string'));
set(handles.slider4,'value',z/250);

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function edit4_Callback(hObject, eventdata, handles)

global theta;
theta=str2num(get(handles.edit4,'string'));
set(handles.slider5,'value',(theta+90)/180);

Global_Para;
if IK_FinalPos==1,
    IK_FinalPosition;
else
end
function edit5_Callback(hObject, eventdata, handles)

global q11;
q11=str2num(get(handles.edit5,'string'));
set(handles.slider7,'value',(q11+360)/720);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function edit6_Callback(hObject, eventdata, handles)

global q12;
q12=str2num(get(handles.edit6,'string'));
set(handles.slider8,'value',q12/45);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function edit7_Callback(hObject, eventdata, handles)

global q21;
q21=str2num(get(handles.edit7,'string'));
set(handles.slider10,'value',(q21+360)/720);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function edit8_Callback(hObject, eventdata, handles)

global q22;
q22=str2num(get(handles.edit8,'string'));
set(handles.slider11,'value',q22/45);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end

% Input the number of trajection points
function edit_Traj_Callback(hObject, eventdata, handles)

global NumTrajPoints_num;
NumTrajPoints_num=str2num(get(handles.edit_Traj,'string'));

if NumTrajPoints_num==1;
    set(handles.pushbutton3,'enable','off');
else
    set(handles.pushbutton3,'enable','on');
end

% Input parameters of FK
function edit10_Callback(hObject, eventdata, handles)

global q0;
q0=str2num(get(handles.edit10,'string'));
set(handles.slider6,'value',(q0+180)/360);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function edit13_Callback(hObject, eventdata, handles)

global q14;
q14=str2num(get(handles.edit13,'string'));
set(handles.slider9,'value',q14/90);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end
function edit16_Callback(hObject, eventdata, handles)

global q23;
q23=str2num(get(handles.edit16,'string'));
set(handles.slider12,'value',q23/90);

Global_Para;
if choice==1
    if IK_FinalPos==1,
    IK_FinalPosition;
    else
    end
else
    if FK_FinalPos==1,
    FK_FinalPosition;
    else
    end
end

% Choose the configuration
function edit_RCB_Callback(hObject, eventdata, handles)

global SolutionRow; 
SolutionRow=str2num(get(handles.edit_RCB,'string'));

% Input the step of simulation
function edit35_Callback(hObject, eventdata, handles)

Global_Para;
n=length(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat);
step=str2num(get(handles.edit35,'string'));
set(handles.slider1,'value',step/n);
ReconbotANI(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(step,:)); 



%---------------------------------------------CallbackFcn of popupmenue------------------------------------------%

%choose Mode to decide the enable of input edit
function popupmenu1_Callback(hObject, eventdata, handles)

global Mode;
global choice;
Mode=get(handles.popupmenu1,'value');

if choice==1
    
    switch Mode
        case 1        
            set(handles.text4,'enable','off'); set(handles.text5,'enable','off'); set(handles.text6,'enable','off'); set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
            set(handles.edit4,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
            set(handles.text1,'string','X[-L1/2,L1/2]'); set(handles.text2,'string','Y[-L1/2,L1/2]');set(handles.text3,'string','Z>0');
            set(handles.slider5,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
            
        case 2
            set(handles.text5,'enable','off'); set(handles.text6,'enable','off'); set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
            set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
            set(handles.text1,'string','X[-L1/2,L1/2]'); set(handles.text2,'string','Y[-L1/2,L1/2]');set(handles.text3,'string','Z>0'); set(handles.text4,'string','?(-90,90)');
            set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
        
        case 3
            set(handles.text1,'enable','off'); set(handles.text2,'enable','off'); set(handles.text6,'enable','off'); set(handles.text7,'enable','off');
            set(handles.edit1,'enable','off'); set(handles.edit2,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off');
            set(handles.text3,'string','Z>0'); set(handles.text4,'string','?(-90,90)'); set(handles.text5,'?11:SingularityA1C1');
            set(handles.slider2,'enable','off'); set(handles.slider3,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off');
        
        case 4
           set(handles.text1,'enable','off'); set(handles.text2,'enable','off'); set(handles.text5,'enable','off'); set(handles.text6,'enable','off'); set(handles.text8,'enable','off');
           set(handles.edit1,'enable','off'); set(handles.edit2,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.text3,'string','Z>0'); set(handles.text4,'string','?(-90,90)'); set(handles.text6,'?21:SingularityA2C2');
           set(handles.slider2,'enable','off'); set(handles.slider3,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider11,'enable','off');
        
        case 5
           set(handles.text1,'enable','off'); set(handles.text2,'enable','off'); set(handles.text4,'enable','off'); set(handles.text6,'enable','off'); set(handles.text8,'enable','off');
           set(handles.edit1,'enable','off'); set(handles.edit2,'enable','off'); set(handles.edit4,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.text3,'string','Z>0'); 
           set(handles.slider2,'enable','off'); set(handles.slider3,'enable','off'); set(handles.slider5,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider11,'enable','off');
        
        case 6
           set(handles.text5,'enable','off'); set(handles.text6,'enable','off'); set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
           set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.text1,'string','X[-L2,L2]'); set(handles.text2,'string','Y[-L2,L2]');set(handles.text3,'string','Z[0,2*L2]'); set(handles.text4,'string','?(-90,90)');
           set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
        
        case 7
           set(handles.text1,'enable','off'); set(handles.text2,'enable','off'); set(handles.text6,'enable','off'); set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
           set(handles.edit1,'enable','off'); set(handles.edit2,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.text3,'string','Z[0,2*L2]'); set(handles.text4,'string','?(-90,90)'); set(handles.text5,'string','?11(-360,360)');
           set(handles.slider2,'enable','off'); set(handles.slider3,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');

        case 8
           set(handles.text2,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.edit2,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
           set(handles.text1,'string','X[-L2,L2]'); set(handles.text3,'string','Z[0,2*L2]'); set(handles.text4,'string','?(-90,90)');
           set(handles.slider3,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
        
        case 9
          set(handles.text2,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
          set(handles.edit2,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
          set(handles.text1,'string','X[-L2,L2]'); set(handles.text3,'string','Z[0,2*L2]'); set(handles.text4,'string','?(-90,90)');
          set(handles.slider3,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider8,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
        
        case 10
          set(handles.text3,'enable','off'); set(handles.text4,'enable','off'); set(handles.text5,'enable','off'); set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
          set(handles.edit3,'enable','off'); set(handles.edit4,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
          set(handles.text1,'string','X[-L1/2,L1/2]'); set(handles.text2,'string','Y[-L1/2,0]'); set(handles.text6,'string','?12');
          set(handles.slider4,'enable','off'); set(handles.slider5,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
        
        case 11 
          set(handles.text3,'enable','off'); set(handles.text4,'enable','off'); set(handles.text5,'enable','off'); set(handles.text5,'enable','off'); set(handles.text7,'enable','off'); 
          set(handles.edit3,'enable','off'); set(handles.edit4,'enable','off'); set(handles.edit5,'enable','off'); set(handles.edit6,'enable','off'); set(handles.edit7,'enable','off');
          set(handles.text1,'string','X(-L1/2,L1/2)'); set(handles.text2,'string','Y(0,L1/2)'); 
          set(handles.slider4,'enable','off'); set(handles.slider5,'enable','off'); set(handles.slider7,'enable','off'); set(handles.slider10,'enable','off');
        
        case 12
          set(handles.text1,'enable','off'); set(handles.text2,'enable','off'); set(handles.text3,'enable','off'); set(handles.text4,'enable','off');
          set(handles.edit1,'enable','off'); set(handles.edit2,'enable','off'); set(handles.edit3,'enable','off'); set(handles.edit4,'enable','off');
          set(handles.text5,'string','?11(-360,360)'); set(handles.text6,'string','?12(0,45)'); set(handles.text7,'string','?21(-360,360)'); set(handles.text8,'string','?22(0,45)');
          set(handles.slider2,'enable','off'); set(handles.slider3,'enable','off'); set(handles.slider4,'enable','off'); set(handles.slider5,'enable','off');
          
    end
    
elseif choice==2
  
    switch Mode
    
    case {1,2,3,4,5}
        set(handles.edit13,'enable','off'); set(handles.edit8,'enable','off'); 
        set(handles.text13,'enable','off'); set(handles.text8,'enable','off'); 
        set(handles.slider9,'enable','off'); set(handles.slider11,'enable','off');
    case {6,7,9}
        set(handles.edit7,'enable','off'); set(handles.edit8,'enable','off');
        set(handles.text7,'enable','off'); set(handles.text8,'enable','off');
        set(handles.slider10,'enable','off'); set(handles.slider11,'enable','off');
    case 8
        set(handles.edit7,'enable','off'); set(handles.edit16,'enable','off');
        set(handles.text7,'enable','off'); set(handles.text16,'enable','off');
        set(handles.slider10,'enable','off'); set(handles.slider12,'enable','off');
    case 10
        set(handles.edit13,'enable','off'); set(handles.edit7,'enable','off');
        set(handles.text13,'enable','off'); set(handles.text7,'enable','off');
        set(handles.slider9,'enable','off'); set(handles.slider10,'enable','off');
    case 11
        set(handles.edit5,'enable','off'); set(handles.edit13,'enable','off');
        set(handles.text5,'enable','off'); set(handles.text13,'enable','off');
        set(handles.slider7,'enable','off'); set(handles.slider9,'enable','off');
    case 12
        set(handles.edit13,'enable','off'); set(handles.edit16,'enable','off');
        set(handles.text13,'enable','off'); set(handles.text16 ,'enable','off');
        set(handles.slider9,'enable','off'); set(handles.slider12,'enable','off');
    end

end
function popupmenu2_Callback(hObject, eventdata, handles)

global NumTP;
NumTP=get(handles.popupmenu2,'value');
function popupmenu3_Callback(hObject, eventdata, handles)

global choice;
choice=get(handles.popupmenu3,'value');

if choice==1;
%set the input of IK enable.
   for i=1:8
       EditList{i}=['edit',num2str(i)];
       TextList{i}=['text',num2str(i)];
   end

   for i=1:8
       set(handles.(EditList{i}),'enable','on');
       set(handles.(TextList{i}),'enable','on');
   end
   
   for i=2:5
       SliderList{i}=['slider',num2str(i)];
   end

   for i=2:5
       set(handles.(SliderList{i}),'enable','on');
   end
   
   set(handles.slider7,'enable','on');
   set(handles.slider8,'enable','on');
   set(handles.slider10,'enable','on');
   
% set the input of IK not enable
   set(handles.edit10,'enable','off');
   set(handles.edit13,'enable','off');
   set(handles.edit16,'enable','off');
   
   set(handles.text10,'enable','off');
   set(handles.text13,'enable','off');
   set(handles.text16,'enable','off');

   set(handles.slider6,'enable','off');
   set(handles.slider9,'enable','off');
   set(handles.slider12,'enable','off');
else
% set the input of IK not enable
      
    for i=1:8
        EditList{i}=['edit',num2str(i)];
        TextList{i}=['text',num2str(i)];
    end

    for i=1:8
        set(handles.(EditList{i}),'enable','off');
        set(handles.(TextList{i}),'enable','off');
    end
    
   for i=2:5
       SliderList{i}=['slider',num2str(i)];
   end

   for i=2:5
       set(handles.(SliderList{i}),'enable','off');
   end

% set the input of FK enable
    set(handles.edit5,'enable','on');
    set(handles.edit7,'enable','on');
    set(handles.edit8,'enable','on');
    set(handles.edit10,'enable','on');
    set(handles.edit6,'enable','on');
    set(handles.edit13,'enable','on');
    set(handles.edit16,'enable','on');
    
    set(handles.text5,'enable','on');
    set(handles.text7,'enable','on');
    set(handles.text8,'enable','on');
    set(handles.text10,'enable','on');
    set(handles.text6,'enable','on');
    set(handles.text13,'enable','on');
    set(handles.text16,'enable','on');
    
   for i=6:12
       SliderList{i}=['slider',num2str(i)];
   end

   for i=6:12
       set(handles.(SliderList{i}),'enable','on');
   end
   
   set(handles.pushbutton4,'string','FK_Simulation');
   set(handles.pushbutton17,'visible','on');
end
function popupmenu4_Callback(hObject, eventdata, handles)

Global_Para;
choice=get(handles.popupmenu4,'value');

switch choice
    case 2
                %  =============================== 3D Animation =================================
        %HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(:,1) = HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(:,2);
        for i = 1:length(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat)
            ReconbotANI(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat(i,:));
        end
        
    case 3
        
        %  =============================== 3D Animation =================================
        %HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,1) = HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(:,2);
        for i = 1:length(HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat)
            ReconbotANI(HomePos2SelectedEndPos_OutputData_RePlan.JointSpace.q0q1q2_Pos_mat(i,:));
        end
        
end

n=length(HomePos2SelectedEndPos_OutputData_Origin.JointSpace.q0q1q2_Pos_mat);
text{1}=['Please Input the Step: 1~',num2str(n)];
set(handles.text47,'string',text{1});
