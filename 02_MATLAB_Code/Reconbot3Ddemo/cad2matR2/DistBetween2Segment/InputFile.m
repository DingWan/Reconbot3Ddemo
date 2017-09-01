clear all
clf
P1 = [150 30 0];     P2 = [-10 50 -20];
P3 = [10 15 0];     P4 = [61 -15 -40];

tic
[distance, ClosePoint] = DistBetween2Segment(P1, P2, P3, P4);
toc

P12 = [P1;P2];
P34 = [P3;P4];
CP = [ClosePoint{2};ClosePoint{3}];
plot3(P12(:,1),P12(:,2),P12(:,3),'r-');hold on
plot3(P34(:,1),P34(:,2),P34(:,3),'b-');hold on
plot3(CP(:,1),CP(:,2),CP(:,3),'k-o');hold on
axis equal