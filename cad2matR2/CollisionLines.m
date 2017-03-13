% %----------------------------------------------------------------
        % Link connection point for collision check
        %Base Low
        BaseLow_Xaxis = [0, 171, 80;   0, 211, 80;   0, 289, 80;   0, 329, 80];
        BaseLow_Yaxis = [500, 171, 80;   500, 211, 80;   500, 289, 80;   500, 329, 80];
        BaseLow_CollisionCheck = [BaseLow_Xaxis, BaseLow_Yaxis];
        %Base Up
        BaseUP_Xaxis = [0, 140, 43.97;   0, 180, 43.97;   150, 320, 43.97;   190, 320, 43.97];
        BaseUP_Yaxis = [340, 140, 43.97;   340, 180, 43.97;   150, 0, 43.97;   190, 0, 43.97];
        BaseUP_CollisionCheck = [BaseUP_Xaxis, BaseUP_Yaxis];
        %LinkA1C1/A2C2
        %LinkA1C1A2C2_Motor = [0, 176.15, 0;   40.20, 176.15, 0;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
        %LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 76.07;   0, 0, 76.07;];
        LinkA1C1A2C2_Motor = [0, 176.15, 30.57;   40.20, 176.15, 30.57;   40.20, 176.15, 76.07;   0, 176.15, 76.07;];
        LinkA1C1A2C2_Bearing = [0, 0, 0;   40.20, 0, 0;   40.20, 0, 35.02;   0, 0, 35.02;];
        LinkA1C1A2C2_CollisionCheck = [LinkA1C1A2C2_Motor, LinkA1C1A2C2_Bearing];
        %Moving Platform
        MP_Xaxis = [0, 140, 14.26;   0, 150, 14.26;   150, 320, 14.26;   190, 320, 14.26];
        MP_Yaxis = [340, 140, 14.26;   340, 180, 14.26;   150, 0, 14.26;   190, 0, 14.26];
        MP_CollisionCheck = [MP_Xaxis, MP_Yaxis];
% %--------------------------------------------------------------------

%LinkA1C1/A2C2
% plot3(BaseLow_Xaxis(:,1), BaseLow_Xaxis(:,2),BaseLow_Xaxis(:,3),'r-o'); hold on
% plot3(BaseLow_Yaxis(:,1), BaseLow_Yaxis(:,2),BaseLow_Yaxis(:,3),'r-o'); hold on

%Base Up
% plot3(BaseUP_Xaxis(:,1), BaseUP_Xaxis(:,2),BaseUP_Xaxis(:,3),'r-o'); hold on
% plot3(BaseUP_Yaxis(:,1), BaseUP_Yaxis(:,2),BaseUP_Yaxis(:,3),'r-o'); hold on

%LinkA1C1/A2C2
% plot3(LinkA1C1A2C2_Motor(:,1), LinkA1C1A2C2_Motor(:,2), LinkA1C1A2C2_Motor(:,3),'r-o'); hold on
% plot3(LinkA1C1A2C2_Bearing(:,1), LinkA1C1A2C2_Bearing(:,2), LinkA1C1A2C2_Bearing(:,3),'r-o'); hold on

%Moving Platform
plot3(MP_Xaxis(:,1), MP_Xaxis(:,2),MP_Xaxis(:,3),'r-o'); hold on
plot3(MP_Yaxis(:,1), MP_Yaxis(:,2),MP_Yaxis(:,3),'r-o'); hold on