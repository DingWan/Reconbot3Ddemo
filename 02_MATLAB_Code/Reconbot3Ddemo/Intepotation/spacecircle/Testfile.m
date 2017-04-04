clear all
clc
l1 = 110;
StartAngle = 0;
EndAngle = 0;
x = l1 * sin(StartAngle);
y = l1 * cos(StartAngle);
a=[0 0 150];
b=[6 50 130];
c=[100 100 100];
n=50;

jz=[a;b;c];
E = SpaceArcGenerator(a,b,c,n);
x=E(1,:);
y=E(2,:);
z=E(3,:);
plot3(x,y,z,'*r');
xlabel('X-axis');ylabel('Y-axis');zlabel('Z-axis');
grid on;
axis equal