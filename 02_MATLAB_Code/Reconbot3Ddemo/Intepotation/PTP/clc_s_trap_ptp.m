function [X,Y,t]=clc_s_trap_ptp(t_end,x_start,y_start,x_end,y_end,steps)
% function to make a trapezodial accaleration trajectory
delta_X=x_end-x_start;%differenz between start and stop in x-direction 
delta_Y=y_end-y_start;%differenz between start and stop in x-direction 
t_c=t_end/3;%devide the time in three phases of equal lenght accalerate contstant velocity and deceleration
time=linspace(0,t_c,steps);%make time signal 
A=[(2*t_c^6)/6     (2*t_c^5)/5     (2*t_c^4)/4    t_c     
     t_c^5           t_c^4        t_c^3       -1
     5*t_c^4         4*t_c^3      3*t_c^2      0
     20*t_c^3        12*t_c^2      6*t_c       0         ];%this function uses a polynomial of 5. order to create the acceleration / decceleration velocity function__ this equation system results from zero acceleration and velocity at t=0 v_end at t=t_f/3 and that the distanz has to be covered at t=t_end
%solve this equation for x and y
B_x=[delta_X;0;0;0];
B_y=[delta_Y;0;0;0];
sol_x = linsolve(A,B_x);
sol_y = linsolve(A,B_y);
%calculate the position acceleration and velocity signal
x=(sol_x(1)*(time.^6)/6)+(sol_x(2)*(time.^5)/5)+(sol_x(3)*(time.^4)/4)+x_start;
x_d=sol_x(1)*time.^5 +sol_x(2)*time.^4+sol_x(3)*time.^3;
x_dd=5*sol_x(1)*time.^4 + 4*sol_x(2)*time.^3 +3*sol_x(3)*time.^2;
X=[x;x_d;x_dd];
X_lin=[sol_x(4)*time+x(end);sol_x(4)*ones(1,numel(time));zeros(1,numel(time))];
X_brake=[sol_x(4)*time-(x-x_start)+X_lin(1,end);sol_x(4)-x_d;-x_dd];
X=[X,X_lin,X_brake];
y=(sol_y(1)*(time.^6)/6)+(sol_y(2)*(time.^5)/5)+(sol_y(3)*(time.^4)/4)+y_start;
y_d=sol_y(1)*time.^5 +sol_y(2)*time.^4+sol_y(3)*time.^3;
y_dd=5*sol_y(1)*time.^4 + 4*sol_y(2)*time.^3 +3*sol_y(3)*time.^2;
Y=[y;y_d;y_dd];
Y_lin=[sol_y(4)*time+y(end);sol_y(4)*ones(1,numel(time));zeros(1,numel(time))];
Y_brake=[sol_y(4)*time-(y-y_start)+Y_lin(1,end);sol_y(4)-y_d;-y_dd];
Y=[Y,Y_lin,Y_brake];
t=linspace(0,t_end,numel(X(1,:)));
figure;
plot(X(2,:),Y(2,:));
figure;
plot(t,X(2,:));
hold on;
plot(t,X(3,:));
end