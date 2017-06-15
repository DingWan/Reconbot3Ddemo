%% Loading WS Data
load('WS_Arm1.mat');
load('WS_Arm2.mat');
load('WS_Arm3.mat');
load('WS_Robot.mat');


%% plot WS_Robot
tic;
subplot(2,2,1);
% sizeWS_Robot= size(WS_Robot);
% i = 1:sizeWS_Robot(1); plot3(WS_Robot(i,1), WS_Robot(i,2), WS_Robot(i,3),'k*','MarkerSize',5);
pcshow([WS_Robot(:,1), WS_Robot(:,2), WS_Robot(:,3)],'MarkerSize',50);
xlabel('X','fontsize',10);
ylabel('Y','fontsize',10);
zlabel('Z','fontsize',10);
title('WS-Robot');
axis equal;
axis([-5,15,-5,15,-8,15]);
toc;


%% plot WS_Arm1
tic;
subplot(2,2,2);
% sizeWS_Arm1= size(WS_Arm1);
% i = 1:sizeWS_Arm1(1); plot3(WS_Arm1(i,1), WS_Arm1(i,2), WS_Arm1(i,3),'k*','MarkerSize',5);
pcshow([WS_Arm1(:,1), WS_Arm1(:,2), WS_Arm1(:,3)],'MarkerSize',50);
xlabel('X','fontsize',10);
ylabel('Y','fontsize',10);
zlabel('Z','fontsize',10);
title('WS-Arm1');
axis equal;
axis([-5,15,-5,15,-8,15]);
toc;

%% plot WS_Arm2
tic;
subplot(2,2,3);
% sizeWS_Arm2= size(WS_Arm2);
% i = 1:sizeWS_Arm2(1); plot3(WS_Arm2(i,1), WS_Arm2(i,2), WS_Arm2(i,3),'k*','MarkerSize',5);
pcshow([WS_Arm2(:,1), WS_Arm2(:,2), WS_Arm2(:,3)],'MarkerSize',50);
xlabel('X','fontsize',10);
ylabel('Y','fontsize',10);
zlabel('Z','fontsize',10);
title('WS-Arm2');
axis equal;
axis([-5,15,-5,15,-8,15]);
toc;

%% plot WS_Arm3
tic;
subplot(2,2,4);
% sizeWS_Arm3= size(WS_Arm3);
% i = 1:sizeWS_Arm3(1); plot3(WS_Arm3(i,1), WS_Arm3(i,2), WS_Arm3(i,3),'k*','MarkerSize',5);
pcshow([WS_Arm3(:,1), WS_Arm3(:,2), WS_Arm3(:,3)],'MarkerSize',50);
xlabel('X','fontsize',10);
ylabel('Y','fontsize',10);
zlabel('Z','fontsize',10);
title('WS-Arm3');
axis equal;
axis([-5,15,-5,15,-8,15]);
% figure;
% contour = [WS_Arm3(:,1), WS_Arm3(:,2), WS_Arm3(:,3)];
% % K1 = convhull(contour);
% K1 = convhull(contour,'simplify',true);
% trisurf(K1,WS_Arm3(:,1),WS_Arm3(:,2),WS_Arm3(:,3), 'Facecolor','cyan'); axis equal;

toc;
