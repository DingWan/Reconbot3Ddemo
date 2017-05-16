clc
clear
clf

l1 = 2.20;
l2 = 1.4725;
L1 = 2.20;
L2 = 1.4725;
deg = pi/180;

q11q13q21q23 = [pi/3, pi/3, pi/3, pi/3];
q11 = 0.1 * pi/2;
q13 = pi/3;
q21 = 0.5 * pi/2;
q23 = pi/3;

x11 = sin(q11);
y11 = cos(q11);
x13 = sin(q13/2);
y13 = cos(q13/2);
x21 = sin(q21);
y21 = cos(q21);
x23 = sin(q23/2);
y23 = cos(q23/2);
% x1213 =  sin(q12 + q13/2);
% y1213 =  cos(q12 + q13/2);
% x2223 =  sin(q22 + q23/2);
% y2223 =  cos(q22 + q23/2);

% syms x
% C1 * x^8  + C2 * x^7 + C3 * x^6  + C4 * x^5 + C5 * x^4  + C6 * x^3 + C7 * x^2 + C8 * x^1 + C9 = 0;
% x = tan[(q12 + q13/2)/2]
% the coefficient matrix of the 8-degree polynomials equations
% the coefficient matrix of the 8-degree polynomials equation
% CoefficientMatrix8 = [C1, C2, C3, C4, C5, C6, C7, C8, C9];

A1 = l2 * x11 * 2 * y13;
A2 = l2 * x21 * 2 * y23;
A3 = l2 * y11 * 2 * y13;
A4 = l2 * y21 * 2 * y23;

C1 = (A1^4 + 2*A1^2*A2^2 + 2*A1^2*A3^2 - 4*A1^2*A3*l1 + 2*A1^2*A4^2 + A2^4 + 2*A2^2*A3^2 - 4*A2^2*A3*l1 + 2*A2^2*A4^2 + A3^4 - 4*A3^3*l1 + 2*A3^2*A4^2 + 4*A3^2*l1^2 - 4*A3*A4^2*l1 + A4^4)*y23^4 + (- 4*A1^2*A2^2 + 8*A1*A2*A3*A4 - 8*A1*A2*A4*l1 - 4*A3^2*A4^2 + 8*A3*A4^2*l1 - 4*A4^2*l1^2)*y23^2;
C2 = 0;
C3 = (- 8*A1^2*A2^2 - 8*A1^2*A4^2 - 8*A2^4 - 8*A2^2*A3^2 + 16*l1*A2^2*A3 - 16*A2^2*A4^2 - 8*A3^2*A4^2 + 16*l1*A3*A4^2 - 8*A4^4)*y13^2*y23^2 + (16*A1^2*A2^2 - 32*A1*A2*A3*A4 + 32*A1*A2*A4*l1 + 16*A3^2*A4^2 - 32*A3*A4^2*l1 + 16*A4^2*l1^2)*y13^2 + (- 4*A1^4 - 8*A1^2*A3^2 + 8*l1*A1^2*A3 + 4*A2^4 - 8*l1*A2^2*A3 + 8*A2^2*A4^2 - 4*A3^4 + 8*l1*A3^3 - 8*l1*A3*A4^2 + 4*A4^4)*y23^4 + (- 16*A4^2*l1^2 + 16*A3*A4^2*l1 - 16*A1*A2*A4*l1)*y23^2;
C4 = 0;
C5 = (16*A2^4 + 32*A2^2*A4^2 + 16*A4^4)*y13^4 + (16*A1^2*A2^2 + 16*A1^2*A4^2 - 16*A2^4 + 16*A2^2*A3^2 - 32*A2^2*A4^2 + 16*A3^2*A4^2 - 16*A4^4)*y13^2*y23^2 + (- 32*A1^2*A2^2 + 64*A1*A2*A3*A4 - 32*A3^2*A4^2 + 32*A4^2*l1^2)*y13^2 + (6*A1^4 - 4*A1^2*A2^2 + 12*A1^2*A3^2 - 4*A1^2*A4^2 + 6*A2^4 - 4*A2^2*A3^2 + 12*A2^2*A4^2 + 6*A3^4 - 4*A3^2*A4^2 - 8*A3^2*l1^2 + 6*A4^4)*y23^4 + (8*A1^2*A2^2 - 16*A1*A2*A3*A4 + 8*A3^2*A4^2 - 24*A4^2*l1^2)*y23^2;
C6 = 0;
C7 = (- 8*A1^2*A2^2 - 8*A1^2*A4^2 - 8*A2^4 - 8*A2^2*A3^2 - 16*l1*A2^2*A3 - 16*A2^2*A4^2 - 8*A3^2*A4^2 - 16*l1*A3*A4^2 - 8*A4^4)*y13^2*y23^2 + (16*A1^2*A2^2 - 32*A1*A2*A3*A4 - 32*A1*A2*A4*l1 + 16*A3^2*A4^2 + 32*A3*A4^2*l1 + 16*A4^2*l1^2)*y13^2 + (- 4*A1^4 - 8*A1^2*A3^2 - 8*l1*A1^2*A3 + 4*A2^4 + 8*l1*A2^2*A3 + 8*A2^2*A4^2 - 4*A3^4 - 8*l1*A3^3 + 8*l1*A3*A4^2 + 4*A4^4)*y23^4 + (- 16*A4^2*l1^2 - 16*A3*A4^2*l1 + 16*A1*A2*A4*l1)*y23^2;
C8 = 0;
C9 = (A1^4 + 2*A1^2*A2^2 + 2*A1^2*A3^2 + 4*A1^2*A3*l1 + 2*A1^2*A4^2 + A2^4 + 2*A2^2*A3^2 + 4*A2^2*A3*l1 + 2*A2^2*A4^2 + A3^4 + 4*A3^3*l1 + 2*A3^2*A4^2 + 4*A3^2*l1^2 + 4*A3*A4^2*l1 + A4^4)*y23^4 + (- 4*A1^2*A2^2 + 8*A1*A2*A3*A4 + 8*A1*A2*A4*l1 - 4*A3^2*A4^2 - 8*A3*A4^2*l1 - 4*A4^2*l1^2)*y23^2;


% the coefficient matrix of the 8-degree polynomials equation
CoefficientMatrix8 = [C1, C2, C3, C4, C5, C6, C7, C8, C9];

% calculate the root of the 8-degree polynomials equation
x = roots(CoefficientMatrix8);

% get the value of real number (+/-)
x = x(imag(x) == 0);
NumRealq12 = length (x);

% the final value of "+/-theta13"
% the programm should judge the sign of the value

q12all = 2 * atan(x) - q13/2 ;
%% ------------------Obtain all of the correct values and assign to q13q23-------------------------
% numbers of i and j are used to count the possible values that satisfy the
% condiation of C1z = C2z; And then, assign all the possible values to
% matrix q13q23;

i = 1;
for Numq12 = 1:NumRealq12
    
    q12SingleValue = q12all(Numq12);
    j = 0;
    %% -------------- Calculate the value of q23 (theta23) ----------------
    % Using the following function to calculate the value of q23
    % L2 * cos(q23)^2 + (D1 * cos(q13) + D2) * cos(q23) + (L2 * cos(q13)^2 + E1 * cos(q13) + E2) = 0
    % the coefficient matrix of the 2-degree polynomials equations
    
    x1213 =  sin(q12SingleValue + q13/2);
    y1213 =  cos(q12SingleValue + q13/2);
    if y23 ~= 0
        x2223 = y13 * x1213 / y23;
    end
    
    % calculate the root of the 2-degree polynomials equation
    q22all = asin(x2223) - q23/2;    
    %q22all * 180 / pi
    
    for Numq22 = 1:length(q22all)
        
        q22SingleValue = q22all(Numq22);
        %% -----------------------Get the output values of Moving Platform-----------------------
        %%--------------------Calculate the position of Ai Bi Ci------------------
        A1 = [0, -L1/2, 0];
        B1 = [L2 * cos(q12SingleValue) * sin(q11), -L1/2 - L2 * cos(q12SingleValue) * cos(q11), L2 * sin(q12SingleValue)];
        C1 = [L2 * (cos(q12SingleValue) + cos(q12SingleValue + q13)) * sin(q11), -L1/2 - L2 * (cos(q12SingleValue) + cos(q12SingleValue + q13)) * cos(q11), L2 * (sin(q12SingleValue) + sin(q12SingleValue + q13))];
        
        A2 = [0, L1/2, 0];
        B2 = [- L2 * cos(q22SingleValue) * sin(q21), L1/2 + L2 * cos(q22SingleValue) * cos(q21), L2 * sin(q22SingleValue)];
        C2 = [- L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * sin(q21), L1/2 + L2 * (cos(q22SingleValue) + cos(q22SingleValue + q23)) * cos(q21), L2 * (sin(q22SingleValue) + sin(q22SingleValue + q23))];
        %%------------------------------------------------------------------------
        
        % Get rid of the undesired points
        if abs(C1(3) - C2(3)) > 1e-8 || norm(C1 - C2) - L1 > 1e-8 
            continue;
        end
        
        % assign the values to q13q23
        q12 = q12SingleValue;
        q22 = q22SingleValue;
        
        % Calculate the center point of moving platform
        p(i+j,1:3) = (C1 + C2) / 2;
        C1C2 = C2 - C1;
        yaxis = [0, 1, 0];
        % Judge the direction of the rotation around Z-axis
        directionC1C2Xxaxis = cross(C1C2, yaxis);
        if directionC1C2Xxaxis(3) < 0
            gamma(i+j) = -acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
        else
            gamma(i+j) = acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
        end
        gamma(i+j) * 180 / pi;
        p(i+j,4:6) = [0, 0, gamma(i+j)];
        ABC(i+j,:) = [A1, B1, C1, A2, B2, C2];
        
        %%-------------------------q14q15 and q24q25------------------------------
        % Calculate the angles of q14q15 and q24q25
        q14 = pi - q12 - q13;% Actually, it should be:q14 = q12 + q13 - pi/2;
        q15 = -q11 - gamma(i+j);
        q24 = pi - q22 - q23;%Actually, it should be:q14 = q22 + q23 - pi/2;
        q25 = q21 - gamma(i+j);
        %%------------------------------------------------------------------------
        %%-------------------------q11-q15 and q21-q25------------------------------
        q1q2(i+j,:) = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];
        
        j = j + 1;
        %% --------------------Plot the mechanism Ai Bi Ci------------------
        PA1B1C1x = [A1(1), B1(1), C1(1)];
        PA1B1C1y = [A1(2), B1(2), C1(2)];
        PA1B1C1z = [A1(3), B1(3), C1(3)];
        plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
        
        PA2B2C2x = [A2(1), B2(1), C2(1)];
        PA2B2C2y = [A2(2), B2(2), C2(2)];
        PA2B2C2z = [A2(3), B2(3), C2(3)];
        plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
        
        PC1C2x = [C1(1), C2(1)];
        PC1C2y = [C1(2), C2(2)];
        PC1C2z = [C1(3), C2(3)];
        plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
        
        PA1A2x = [A1(1), A2(1)];
        PA1A2y = [A1(2), A2(2)];
        PA1A2z = [A1(3), A2(3)];
        
        plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
        %grid on;
        %axis equal;
        xlabel('x');
        ylabel('y');
        zlabel('z');
        axis equal;
    end
    i = i + j;
end

%----------------- plot xyz axes of base point --------------
x_axis = [0.5 0 0];
y_axis = [0 0.5 0];
z_axis = [0 0 0.5];
OP= [0 0 0];
xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
i = 1:2;
plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-r');
i = 3:4;
plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-g');
i = 5:6;
plot3(xyz(i,1),xyz(i,2),xyz(i,3),'-b');
%-----------------------------------------------------------

%%Here is used to judge the existance of the solution
if i == 1
    display('No solution for this input, Calculation process is stopped');
    p = [];
    ABC = [];
    q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
end
