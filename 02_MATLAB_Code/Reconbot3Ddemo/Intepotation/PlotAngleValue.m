figure(2)
clf
q11q12q14_q21q22q23_Angle = q11q12q14_q21q22q23 * 180/pi;
%------ Show Center point of Moving Platform -------
BaseLow_CenterPointBearing = [250, 250, 83.5];
BaseUP_CenterPointBearing = [160, 110, 60.44];
BaseJointA1C1A2C2_CenterPointMotor = [34.5, 48.1, 45.5];
BaseJointA1C1A2C2_CenterPointSidePlate = [58.5, 48.1, 22];
% in World frame
Displacement = [250,250,83.5+60.44+(45.5-22)] / 1000;
MP_Pos = MP_Pos_mat(1:3) / 1000 + Displacement;

%% Moving Platform
subplot(3,2,[1 3 5]);
% in local frame
plot3(MP_Pos_mat(:,1),MP_Pos_mat(:,2),MP_Pos_mat(:,3),'b-', 'Linewidth', 2); hold on
%----------------- plot xyz axes of base point --------------
x_axis = [20 0 0] / 1000;
y_axis = [0 20 0] / 1000;
z_axis = [0 0 20] / 1000;
OP= [0 0 0];
xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
j = 1:2;
plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-r','LineWidth',2); hold on
j = 3:4;
plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-g','LineWidth',2); hold on
j = 5:6;
plot3(xyz(j,1),xyz(j,2),xyz(j,3),'-b','LineWidth',2); hold on
title('Moving Platform Trajectory')
grid on
axis equal

%% Joint Motion
subplot(3,2,2);
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,1),'r'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,4),'g'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,7),'b'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,10),'m'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,13),'k'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,16),'c'); hold on
title('Angle: q11(R),q12(G),q14(B),q21(M),q22(B),q23(C)')
grid on

subplot(3,2,4);
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,2),'r'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,5),'g'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,8),'b'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,11),'m'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,14),'k'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,17),'c'); hold on
title('AngularVelocity: q11(R),q12(G),q14(B),q21(M),q22(B),q23(C)')
grid on

subplot(3,2,6);
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,3),'r'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,6),'g'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,9),'b'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,12),'m'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,15),'k'); hold on
plot(Time_mat(:,1),q11q12q14_q21q22q23_Angle(:,18),'c'); hold on
title('AngularAcceleration: q11(R),q12(G),q14(B),q21(M),q22(B),q23(C)')
grid on