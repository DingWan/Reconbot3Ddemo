clc

clear
% circle_origin is the world frame below; 
% X-axis: 
% Y-axis: A2 to A1
% Z-axis: Gravity direction
circle_origin = [0 0 0];
World_Origin = [-228.752 204.351 88.467];
World_Origin_New = [-228.714  204.350  88.269];
New_displacement = [0.0380    0.0010    -0.1980];

%% Motor 4
M4_Zaxis = [114.986 0.047 -85.074];
LeftMiddleJoint = [262.714  38.699  58.781];

%% Motor 5
M5_Yaxis = [115.095 73.988 -58.812];

%% Motor 3
M3_Yaxis = [114.914  -87.137 -58.175];


%% Motor 1
M1_Zaxis = [-115.044  -0.033  -85.111];
M1_Zaxis_NewWO = [-115.070  0.563  -85.244] + New_displacement;

%% Motor 2
M2_Yaxis_1 = [-114.964  -74.095  -58.598];
M2_Yaxis_2 = [-114.298 -73.501 -58.929];
M2_Yaxis_NewWO = [-114.190 -73.586 -58.858]  + New_displacement;

%% Motor 6
M6_2_MiddleScrew = [-114.531  74.128  -58.249];
M6_1 = [-263.224  52.756  -58.783];

M6_Yaxis_NewWO_1 = [ -262.816 53.245 -58.025 ] + New_displacement;
M6_Yaxis_NewWO_180degree = [-263.021 52.849 -59.013] + New_displacement;
M6_Yaxis_NewWO_2 = [-262.640 53.369 -58.275] + New_displacement;
M6_Yaxis_NewWO_3 = [-262.672  53.584 -58.284] + New_displacement;

%% 
UpJoint_MP_90degree = [ -114.639 -0.638 -133.166];
UpJoint_UpLink_NewWO_180degree = [-143.751  67.644 -58.439];

UpJoint_UpLink90degree_NewWO = [-263.214 22.003 -205.227] + New_displacement;
UpJoint_UpLink90degree_NewWO_2 = [-263.433 22.864 -205.124] + New_displacement;
UpJoint_UpLink90degree_NewWO_3 = [-263.315 23.553 -205.201] + New_displacement;

%% Link Length
B1C1 = (-205.3990 -205.3220 -205.4250)/3 - (-58.4820 -58.4730 -58.2230)/3
A1B1 = (-262.816 -262.640 -262.672)/3 - (-115.044 -115.070 -114.964)/3
Delta_x_B1C1 = (-262.816 -262.640 -262.672)/3 - (-263.214 -263.433 -263.315)/3
B1C1 = -sqrt(B1C1^2 + Delta_x_B1C1^2)

A2B2 = 262.714 - (114.986 + 115.095 + 114.914)/3

A1A2 = 115.095 + 115.044