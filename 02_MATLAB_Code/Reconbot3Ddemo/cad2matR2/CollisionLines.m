% %----------------------------------------------------------------
% Link connection point for collision check
%Base Low
BaseLow_alongXaxis = [0, 171, 80;   0, 211, 80;   0, 289, 80;   0, 329, 80];
BaseLow_alongYaxis = [500, 171, 80;   500, 211, 80;   500, 289, 80;   500, 329, 80];
BaseLow_CollisionCheck = [BaseLow_alongXaxis, ones(4, 1); BaseLow_alongYaxis, ones(4, 1)];

%Base Up
BaseUP_alongXaxis = [0, 90, 10;   0, 130, 10;   140, 220, 10;   180, 220, 10];
BaseUP_alongYaxis = [320, 90, 10;   320, 130, 10;   140, 0, 10;   180, 0, 10];
BaseUP_CollisionCheck = [BaseUP_alongXaxis, ones(4, 1); BaseUP_alongYaxis, ones(4, 1)];

%LowUpLinkA1C1
% Method I: Large potection area : Cuboid ≥§∑ΩÃÂ
%LinkA1C1A2C2_Motor = [0, 176.15, 0;   40.20, 176.15, 0;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
%LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 76.07;   0, 0, 76.07;];
% Method II: Small potection area : Oblique Prism –±¿‚÷˘
LowUpLinkA1C1_Motor =   [0, 176.15, 0;  40.20, 176.15, 0;   40.20, 176.15, 50;   0, 176.15, 50;];
LowUpLinkA1C1_Bearing = [0, 0, 40;    40.20, 0, 40;   40.20, 0, 80;        0, 0, 80;];
LinkA1C1_CollisionCheck = [LowUpLinkA1C1_Motor, ones(4, 1); LowUpLinkA1C1_Bearing, ones(4, 1)];

% LowLinkA2C2
% Method II: Small potection area : Oblique Prism –±¿‚÷˘
LowLinkA2C2_Motor = [0, 176.15, 0;  40.20, 176.15, 0;   40.20, 176.15, 50;   0, 176.15, 50;];
LowLinkA2C2_Bearing = [0, 0, 40;    40.20, 0, 40;   40.20, 0, 75;        0, 0, 75;];
LowLinkA2C2_CollisionCheck = [LowLinkA2C2_Motor, ones(4, 1); LowLinkA2C2_Bearing, ones(4, 1)];

% UpLinkA2C2
% Method II: Small potection area : Oblique Prism –±¿‚÷˘
UpLinkA2C2_Motor = [0, 176.15, 0;  40.20, 176.15, 0;   40.20, 176.15, 55;   0, 176.15, 55;];
UpLinkA2C2_Bearing = [0, 0, 0;    40.20, 0, 0;   40.20, 0, 20;        0, 0, 20;];
UpLinkA2C2_CollisionCheck = [UpLinkA2C2_Motor, ones(4, 1); UpLinkA2C2_Bearing, ones(4, 1)];

%Moving Platform
MP_alongXaxis = [0, 0, 0;   0, 40, 0;   0, 0, 40;   0, 40, 40];
MP_alongYaxis = [320, 0, 0;   320, 40, 0;   320, 0, 40;   320, 40, 40];
MP_CollisionCheck = [MP_alongXaxis, ones(4, 1); ];
% %--------------------------------------------------------------------

%BaseLow
% plot3(BaseLow_alongXaxis(:,1), BaseLow_alongXaxis(:,2),BaseLow_alongXaxis(:,3),'r-o'); hold on
% plot3(BaseLow_alongYaxis(:,1), BaseLow_alongYaxis(:,2),BaseLow_alongYaxis(:,3),'r-o'); hold on

%Base Up
% plot3(BaseUP_alongXaxis(:,1), BaseUP_alongXaxis(:,2),BaseUP_alongXaxis(:,3),'r-o'); hold on
% plot3(BaseUP_alongYaxis(:,1), BaseUP_alongYaxis(:,2),BaseUP_alongYaxis(:,3),'r-o'); hold on

%LowUpLinkA1C1
% plot3(LowUpLinkA1C1_Motor(:,1), LowUpLinkA1C1_Motor(:,2), LowUpLinkA1C1_Motor(:,3),'r-o'); hold on
% plot3(LowUpLinkA1C1_Bearing(:,1), LowUpLinkA1C1_Bearing(:,2), LowUpLinkA1C1_Bearing(:,3),'r-o'); hold on

%LowLinkA2C2
% plot3(LowLinkA2C2_Motor(:,1), LowLinkA2C2_Motor(:,2), LowLinkA2C2_Motor(:,3),'r-o'); hold on
% plot3(LowLinkA2C2_Bearing(:,1), LowLinkA2C2_Bearing(:,2), LowLinkA2C2_Bearing(:,3),'r-o'); hold on

%UpLinkA2C2
% plot3(UpLinkA2C2_Motor(:,1), UpLinkA2C2_Motor(:,2), UpLinkA2C2_Motor(:,3),'r-o'); hold on
% plot3(UpLinkA2C2_Bearing(:,1), UpLinkA2C2_Bearing(:,2), UpLinkA2C2_Bearing(:,3),'r-o'); hold on

%Moving Platform
plot3(MP_alongXaxis(:,1), MP_alongXaxis(:,2),MP_alongXaxis(:,3),'r-o'); hold on
plot3(MP_alongYaxis(:,1), MP_alongYaxis(:,2),MP_alongYaxis(:,3),'r-o'); hold on