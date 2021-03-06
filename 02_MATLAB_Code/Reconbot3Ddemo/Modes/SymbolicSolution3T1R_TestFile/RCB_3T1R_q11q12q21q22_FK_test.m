clc
clear
clf
l1 = 230.0692;
l2 = 146.25;
L1 = 230.0692;
L2 = 146.25;

% l1 = 2.20;
% l2 = 1.4725;
% L1 = 2.20;
% L2 = 1.4725;
% deg = pi/180;

q11q12q21q22 = [0.5*pi/2, 1.0*pi/3, 0.8*pi/2, 1.0*pi/6];
% q11q12q21q22 = [1.35283894173723    1.51568066620560    0.771619248250125    1.01963972090195];
q11 = q11q12q21q22(1);
q12 = q11q12q21q22(2);
q21 = q11q12q21q22(3);
q22 = q11q12q21q22(4);

x11 = sin(q11);
y11 = cos(q11);
x12 = sin(q12);
y12 = cos(q12);
x21 = sin(q21);
y21 = cos(q21);
x22 = sin(q22);
y22 = cos(q22);
% x1213 =  sin(q12 + q13);
% y1213 =  cos(q12 + q13);
% x2223 =  sin(q22 + q23);
% y2223 =  cos(q22 + q23);

%% Hand Calculate
%%----------------------------------------------------------------------------            
% the Substitutional Variables
F = cos(q11) * cos(q21) - sin(q11) * sin(q21);
G = sin(q12) - sin(q22);

D1 = 2 * L2 * F;
D2 = 2 * (L2 * cos(q22) + L1 * cos(q21) + L2 * F * cos(q12));

E1 = 2 * (L2 * cos(q12) + L1 * cos(q11) + L2 * F * cos(q22));
E2 = L2 * cos(q12)^2 + 2 * (L2 * F * cos(q22) + L1 * cos(q11)) * cos(q12) + L2 * cos(q22)^2 + 2 * L1 * cos(q21) * cos(q22);

H1 = L2^2;
H2 = 2 * L2 * E1;
H3 = 2 * L2^2 + E1^2 + 2 * L2 * E2 - D1^2;
H4 = 2 * (L2 * E1 + E1 * E2 - D1 * D2);
H5 = L2^2 + 2 * L2 * E2 + E2^2 - D2^2;

I1 = 2 * L2^2 - D1^2;
I2 = 2 * (L2 * E1 - D1 * D2);
I3 = 2 * L2^2 + 2 * L2 * E2 - D2^2;

%%----------------------------Simplified version----------------------------
J1 = L2^2 * G^4 - G^2 * (I1 - I2 + I3) + H1 - H2 + H3 - H4 + H5;
J2 = L2^2 * 8 * G^3 - 4 * G * (I1 - I2 + I3);
J3 = L2^2 * (4 * G^4 + 24 * G^2) - (4 * (I1 - I2 + I3) + 2 * G^2 * (2 * I3 - I2)) - 4 * H1 + 2 * H2 - 2 * H4 + 4 * H5;
J4 = L2^2 * (24 * G^3 + 32 * G) - 4 * G * (-I1 - I2 + 3 * I3);
J5 = L2^2 * (6 * G^4 + 48 * G^2 + 16) - (8 * (I3 - I1) + 2 * G^2 * (3 * I3 - I1)) + 6 * H1 - 2 * H3 + 6 * H5;
J6 = L2^2 * (24 * G^3 + 32 * G) - 4 * G * (-I1 + I2 + 3 * I3);
J7 = L2^2 * (4 * G^4 + 24 * G^2) - (4 * (I1 + I2 + I3) + 2 * G^2 * (I2 + 2 * I3)) - 4 * H1 - 2 * H2 + 2 * H4 + 4 * H5;
J8 = L2^2 * 8 * G^3 - 4 * G * (I1 + I2 + I3);
J9 = L2^2 * G^4 - G^2 * (I1 + I2 + I3) + H1 + H2 + H3 + H4 + H5;

% the coefficient matrix of the 8-degree polynomials equation
CoefficientMatrix8_Ji = [J1, J2, J3, J4, J5, J6, J7, J8, J9];

% calculate the root of the 8-degree polynomials equation
x = roots(CoefficientMatrix8_Ji);
%%----------------------------------------------------------------------------  
% J1 * x.^8  + J2 * x.^7 + J3 * x.^6  + J4 * x.^5 + J5 * x.^4  + J6 * x.^3 + J7 * x.^2 + J8 * x.^1 + J9;

%% Symbolic Solution
% syms x
% C1 * x^8  + C2 * x^7 + C3 * x^6  + C4 * x^5 + C5 * x^4  + C6 * x^3 + C7 * x^2 + C8 * x^1 + C9 = 0;
% x = tan[(q12 + q13/2)/2]
% the coefficient matrix of the 8-degree polynomials equations
% the coefficient matrix of the 8-degree polynomials equation
% CoefficientMatrix8 = [C1, C2, C3, C4, C5, C6, C7, C8, C9];
%%----------------------------------------------------------------------------  
A1 = l2 * x11;
A2 = l2 * x21;
A3 = l2 * x11 * y12 - l2 * x21 * y22;
A4 = l2 * y11;
A5 = l2 * y21;
A6 = l2 * y11 * y12 + l2 * y21 * y22 + l1;

C1 =  (A2^4 + 2*A2^2*A5^2 + A5^4)*x12^4 + (- 4*A2^4 - 8*A2^2*A5^2 - 4*A5^4)*x12^3*x22 + (6*A2^4 + 12*A2^2*A5^2 + 6*A5^4)*x12^2*x22^2 + (2*A1^2*A2^2 - 2*A1^2*A5^2 - 4*A1*A2^2*A3 - 8*A1*A2*A4*A5 + 8*A1*A2*A5*A6 + 4*A1*A3*A5^2 - 2*A2^4 + 2*A2^2*A3^2 - 2*A2^2*A4^2 + 4*A2^2*A4*A6 - 4*A2^2*A5^2 - 2*A2^2*A6^2 + 2*A2^2*l1^2 + 8*A2*A3*A4*A5 - 8*A2*A3*A5*A6 - 2*A3^2*A5^2 + 2*A4^2*A5^2 - 4*A4*A5^2*A6 - 2*A5^4 + 2*A5^2*A6^2 + 2*A5^2*l1^2)*x12^2 + (- 4*A2^4 - 8*A2^2*A5^2 - 4*A5^4)*x12*x22^3 + (- 4*A1^2*A2^2 + 4*A1^2*A5^2 + 8*A1*A2^2*A3 + 16*A1*A2*A4*A5 - 16*A1*A2*A5*A6 - 8*A1*A3*A5^2 + 4*A2^4 - 4*A2^2*A3^2 + 4*A2^2*A4^2 - 8*A2^2*A4*A6 + 8*A2^2*A5^2 + 4*A2^2*A6^2 - 4*A2^2*l1^2 - 16*A2*A3*A4*A5 + 16*A2*A3*A5*A6 + 4*A3^2*A5^2 - 4*A4^2*A5^2 + 8*A4*A5^2*A6 + 4*A5^4 - 4*A5^2*A6^2 - 4*A5^2*l1^2)*x12*x22 + (A2^4 + 2*A2^2*A5^2 + A5^4)*x22^4 + (2*A1^2*A2^2 - 2*A1^2*A5^2 - 4*A1*A2^2*A3 - 8*A1*A2*A4*A5 + 8*A1*A2*A5*A6 + 4*A1*A3*A5^2 - 2*A2^4 + 2*A2^2*A3^2 - 2*A2^2*A4^2 + 4*A2^2*A4*A6 - 4*A2^2*A5^2 - 2*A2^2*A6^2 + 2*A2^2*l1^2 + 8*A2*A3*A4*A5 - 8*A2*A3*A5*A6 - 2*A3^2*A5^2 + 2*A4^2*A5^2 - 4*A4*A5^2*A6 - 2*A5^4 + 2*A5^2*A6^2 + 2*A5^2*l1^2)*x22^2 + A1^4 - 4*A1^3*A3 - 2*A1^2*A2^2 + 6*A1^2*A3^2 + 2*A1^2*A4^2 - 4*A1^2*A4*A6 + 2*A1^2*A5^2 + 2*A1^2*A6^2 - 2*A1^2*l1^2 + 4*A1*A2^2*A3 + 8*A1*A2*A4*A5 - 8*A1*A2*A5*A6 - 4*A1*A3^3 - 4*A1*A3*A4^2 + 8*A1*A3*A4*A6 - 4*A1*A3*A5^2 - 4*A1*A3*A6^2 + 4*A1*A3*l1^2 + A2^4 - 2*A2^2*A3^2 + 2*A2^2*A4^2 - 4*A2^2*A4*A6 + 2*A2^2*A5^2 + 2*A2^2*A6^2 - 2*A2^2*l1^2 - 8*A2*A3*A4*A5 + 8*A2*A3*A5*A6 + A3^4 + 2*A3^2*A4^2 - 4*A3^2*A4*A6 + 2*A3^2*A5^2 + 2*A3^2*A6^2 - 2*A3^2*l1^2 + A4^4 - 4*A4^3*A6 - 2*A4^2*A5^2 + 6*A4^2*A6^2 - 2*A4^2*l1^2 + 4*A4*A5^2*A6 - 4*A4*A6^3 + 4*A4*A6*l1^2 + A5^4 - 2*A5^2*A6^2 - 2*A5^2*l1^2 + A6^4 - 2*A6^2*l1^2 + l1^4;
C2 =  (8*A2^4 + 16*A2^2*A5^2 + 8*A5^4)*x12^3 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x12^2*x22 + (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12*x22^2 + (8*A1^2*A2^2 - 8*A1^2*A5^2 - 16*A1*A2^2*A3 - 32*A1*A2*A4*A5 + 32*A1*A2*A5*A6 + 16*A1*A3*A5^2 - 8*A2^4 + 8*A2^2*A3^2 - 8*A2^2*A4^2 + 16*A2^2*A4*A6 - 16*A2^2*A5^2 - 8*A2^2*A6^2 + 8*A2^2*l1^2 + 32*A2*A3*A4*A5 - 32*A2*A3*A5*A6 - 8*A3^2*A5^2 + 8*A4^2*A5^2 - 16*A4*A5^2*A6 - 8*A5^4 + 8*A5^2*A6^2 + 8*A5^2*l1^2)*x12 + (- 8*A2^4 - 16*A2^2*A5^2 - 8*A5^4)*x22^3 + (- 8*A1^2*A2^2 + 8*A1^2*A5^2 + 16*A1*A2^2*A3 + 32*A1*A2*A4*A5 - 32*A1*A2*A5*A6 - 16*A1*A3*A5^2 + 8*A2^4 - 8*A2^2*A3^2 + 8*A2^2*A4^2 - 16*A2^2*A4*A6 + 16*A2^2*A5^2 + 8*A2^2*A6^2 - 8*A2^2*l1^2 - 32*A2*A3*A4*A5 + 32*A2*A3*A5*A6 + 8*A3^2*A5^2 - 8*A4^2*A5^2 + 16*A4*A5^2*A6 + 8*A5^4 - 8*A5^2*A6^2 - 8*A5^2*l1^2)*x22;
C3 =  (4*A2^4 + 8*A2^2*A5^2 + 4*A5^4)*x12^4 + (- 16*A2^4 - 32*A2^2*A5^2 - 16*A5^4)*x12^3*x22 + (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12^2*x22^2 + (16*A2^4 + 8*A2^2*A3^2 - 8*A1*A2^2*A3 + 32*A2^2*A5^2 - 8*A2^2*A6^2 + 8*A4*A2^2*A6 + 8*A2^2*l1^2 - 32*A2*A3*A5*A6 + 16*A4*A2*A3*A5 + 16*A1*A2*A5*A6 - 8*A3^2*A5^2 + 8*A1*A3*A5^2 + 16*A5^4 + 8*A5^2*A6^2 - 8*A4*A5^2*A6 + 8*A5^2*l1^2)*x12^2 + (- 16*A2^4 - 32*A2^2*A5^2 - 16*A5^4)*x12*x22^3 + (- 32*A2^4 - 16*A2^2*A3^2 + 16*A1*A2^2*A3 - 64*A2^2*A5^2 + 16*A2^2*A6^2 - 16*A4*A2^2*A6 - 16*A2^2*l1^2 + 64*A2*A3*A5*A6 - 32*A4*A2*A3*A5 - 32*A1*A2*A5*A6 + 16*A3^2*A5^2 - 16*A1*A3*A5^2 - 32*A5^4 - 16*A5^2*A6^2 + 16*A4*A5^2*A6 - 16*A5^2*l1^2)*x12*x22 + (4*A2^4 + 8*A2^2*A5^2 + 4*A5^4)*x22^4 + (16*A2^4 + 8*A2^2*A3^2 - 8*A1*A2^2*A3 + 32*A2^2*A5^2 - 8*A2^2*A6^2 + 8*A4*A2^2*A6 + 8*A2^2*l1^2 - 32*A2*A3*A5*A6 + 16*A4*A2*A3*A5 + 16*A1*A2*A5*A6 - 8*A3^2*A5^2 + 8*A1*A3*A5^2 + 16*A5^4 + 8*A5^2*A6^2 - 8*A4*A5^2*A6 + 8*A5^2*l1^2)*x22^2 - 4*A1^4 + 8*A1^3*A3 + 8*A1^2*A2^2 - 8*A1^2*A4^2 + 8*A1^2*A4*A6 - 8*A1^2*A5^2 - 8*A1*A2^2*A3 - 32*A1*A2*A4*A5 + 16*A1*A2*A5*A6 - 8*A1*A3^3 + 8*A1*A3*A4^2 + 8*A1*A3*A5^2 - 8*A1*A3*A6^2 + 8*A1*A3*l1^2 - 4*A2^4 - 8*A2^2*A4^2 + 8*A2^2*A4*A6 - 8*A2^2*A5^2 + 16*A2*A3*A4*A5 + 4*A3^4 - 8*A3^2*A4*A6 + 8*A3^2*A6^2 - 8*A3^2*l1^2 - 4*A4^4 + 8*A4^3*A6 + 8*A4^2*A5^2 - 8*A4*A5^2*A6 - 8*A4*A6^3 + 8*A4*A6*l1^2 - 4*A5^4 + 4*A6^4 - 8*A6^2*l1^2 + 4*l1^4;
C4 =  (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12^3 + (- 72*A2^4 - 144*A2^2*A5^2 - 72*A5^4)*x12^2*x22 + (72*A2^4 + 144*A2^2*A5^2 + 72*A5^4)*x12*x22^2 + (- 8*A1^2*A2^2 + 8*A1^2*A5^2 - 16*A1*A2^2*A3 + 32*A1*A2*A4*A5 + 32*A1*A2*A5*A6 + 16*A1*A3*A5^2 + 8*A2^4 + 24*A2^2*A3^2 + 8*A2^2*A4^2 + 16*A2^2*A4*A6 + 16*A2^2*A5^2 - 24*A2^2*A6^2 + 24*A2^2*l1^2 + 32*A2*A3*A4*A5 - 96*A2*A3*A5*A6 - 24*A3^2*A5^2 - 8*A4^2*A5^2 - 16*A4*A5^2*A6 + 8*A5^4 + 24*A5^2*A6^2 + 24*A5^2*l1^2)*x12 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x22^3 + (8*A1^2*A2^2 - 8*A1^2*A5^2 + 16*A1*A2^2*A3 - 32*A1*A2*A4*A5 - 32*A1*A2*A5*A6 - 16*A1*A3*A5^2 - 8*A2^4 - 24*A2^2*A3^2 - 8*A2^2*A4^2 - 16*A2^2*A4*A6 - 16*A2^2*A5^2 + 24*A2^2*A6^2 - 24*A2^2*l1^2 - 32*A2*A3*A4*A5 + 96*A2*A3*A5*A6 + 24*A3^2*A5^2 + 8*A4^2*A5^2 + 16*A4*A5^2*A6 - 8*A5^4 - 24*A5^2*A6^2 - 24*A5^2*l1^2)*x22;
C5 =  (6*A2^4 + 12*A2^2*A5^2 + 6*A5^4)*x12^4 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x12^3*x22 + (36*A2^4 + 72*A2^2*A5^2 + 36*A5^4)*x12^2*x22^2 + (- 4*A1^2*A2^2 + 4*A1^2*A5^2 + 16*A1*A2*A4*A5 + 36*A2^4 + 12*A2^2*A3^2 + 4*A2^2*A4^2 + 72*A2^2*A5^2 - 12*A2^2*A6^2 + 12*A2^2*l1^2 - 48*A2*A3*A5*A6 - 12*A3^2*A5^2 - 4*A4^2*A5^2 + 36*A5^4 + 12*A5^2*A6^2 + 12*A5^2*l1^2)*x12^2 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x12*x22^3 + (8*A1^2*A2^2 - 8*A1^2*A5^2 - 32*A1*A2*A4*A5 - 72*A2^4 - 24*A2^2*A3^2 - 8*A2^2*A4^2 - 144*A2^2*A5^2 + 24*A2^2*A6^2 - 24*A2^2*l1^2 + 96*A2*A3*A5*A6 + 24*A3^2*A5^2 + 8*A4^2*A5^2 - 72*A5^4 - 24*A5^2*A6^2 - 24*A5^2*l1^2)*x12*x22 + (6*A2^4 + 12*A2^2*A5^2 + 6*A5^4)*x22^4 + (- 4*A1^2*A2^2 + 4*A1^2*A5^2 + 16*A1*A2*A4*A5 + 36*A2^4 + 12*A2^2*A3^2 + 4*A2^2*A4^2 + 72*A2^2*A5^2 - 12*A2^2*A6^2 + 12*A2^2*l1^2 - 48*A2*A3*A5*A6 - 12*A3^2*A5^2 - 4*A4^2*A5^2 + 36*A5^4 + 12*A5^2*A6^2 + 12*A5^2*l1^2)*x22^2 + 6*A1^4 - 12*A1^2*A2^2 - 12*A1^2*A3^2 + 12*A1^2*A4^2 + 12*A1^2*A5^2 - 4*A1^2*A6^2 + 4*A1^2*l1^2 + 48*A1*A2*A4*A5 - 16*A1*A3*A4*A6 + 6*A2^4 + 4*A2^2*A3^2 + 12*A2^2*A4^2 + 12*A2^2*A5^2 - 4*A2^2*A6^2 + 4*A2^2*l1^2 - 16*A2*A3*A5*A6 + 6*A3^4 - 4*A3^2*A4^2 - 4*A3^2*A5^2 + 12*A3^2*A6^2 - 12*A3^2*l1^2 + 6*A4^4 - 12*A4^2*A5^2 - 12*A4^2*A6^2 + 4*A4^2*l1^2 + 6*A5^4 + 4*A5^2*A6^2 + 4*A5^2*l1^2 + 6*A6^4 - 12*A6^2*l1^2 + 6*l1^4;
C6 = (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12^3 + (- 72*A2^4 - 144*A2^2*A5^2 - 72*A5^4)*x12^2*x22 + (72*A2^4 + 144*A2^2*A5^2 + 72*A5^4)*x12*x22^2 + (- 8*A1^2*A2^2 + 8*A1^2*A5^2 + 16*A1*A2^2*A3 + 32*A1*A2*A4*A5 - 32*A1*A2*A5*A6 - 16*A1*A3*A5^2 + 8*A2^4 + 24*A2^2*A3^2 + 8*A2^2*A4^2 - 16*A2^2*A4*A6 + 16*A2^2*A5^2 - 24*A2^2*A6^2 + 24*A2^2*l1^2 - 32*A2*A3*A4*A5 - 96*A2*A3*A5*A6 - 24*A3^2*A5^2 - 8*A4^2*A5^2 + 16*A4*A5^2*A6 + 8*A5^4 + 24*A5^2*A6^2 + 24*A5^2*l1^2)*x12 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x22^3 + (8*A1^2*A2^2 - 8*A1^2*A5^2 - 16*A1*A2^2*A3 - 32*A1*A2*A4*A5 + 32*A1*A2*A5*A6 + 16*A1*A3*A5^2 - 8*A2^4 - 24*A2^2*A3^2 - 8*A2^2*A4^2 + 16*A2^2*A4*A6 - 16*A2^2*A5^2 + 24*A2^2*A6^2 - 24*A2^2*l1^2 + 32*A2*A3*A4*A5 + 96*A2*A3*A5*A6 + 24*A3^2*A5^2 + 8*A4^2*A5^2 - 16*A4*A5^2*A6 - 8*A5^4 - 24*A5^2*A6^2 - 24*A5^2*l1^2)*x22;
C7 = (4*A2^4 + 8*A2^2*A5^2 + 4*A5^4)*x12^4 + (- 16*A2^4 - 32*A2^2*A5^2 - 16*A5^4)*x12^3*x22 + (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12^2*x22^2 + (16*A2^4 + 8*A2^2*A3^2 + 8*A1*A2^2*A3 + 32*A2^2*A5^2 - 8*A2^2*A6^2 - 8*A4*A2^2*A6 + 8*A2^2*l1^2 - 32*A2*A3*A5*A6 - 16*A4*A2*A3*A5 - 16*A1*A2*A5*A6 - 8*A3^2*A5^2 - 8*A1*A3*A5^2 + 16*A5^4 + 8*A5^2*A6^2 + 8*A4*A5^2*A6 + 8*A5^2*l1^2)*x12^2 + (- 16*A2^4 - 32*A2^2*A5^2 - 16*A5^4)*x12*x22^3 + (- 32*A2^4 - 16*A2^2*A3^2 - 16*A1*A2^2*A3 - 64*A2^2*A5^2 + 16*A2^2*A6^2 + 16*A4*A2^2*A6 - 16*A2^2*l1^2 + 64*A2*A3*A5*A6 + 32*A4*A2*A3*A5 + 32*A1*A2*A5*A6 + 16*A3^2*A5^2 + 16*A1*A3*A5^2 - 32*A5^4 - 16*A5^2*A6^2 - 16*A4*A5^2*A6 - 16*A5^2*l1^2)*x12*x22 + (4*A2^4 + 8*A2^2*A5^2 + 4*A5^4)*x22^4 + (16*A2^4 + 8*A2^2*A3^2 + 8*A1*A2^2*A3 + 32*A2^2*A5^2 - 8*A2^2*A6^2 - 8*A4*A2^2*A6 + 8*A2^2*l1^2 - 32*A2*A3*A5*A6 - 16*A4*A2*A3*A5 - 16*A1*A2*A5*A6 - 8*A3^2*A5^2 - 8*A1*A3*A5^2 + 16*A5^4 + 8*A5^2*A6^2 + 8*A4*A5^2*A6 + 8*A5^2*l1^2)*x22^2 - 4*A1^4 - 8*A1^3*A3 + 8*A1^2*A2^2 - 8*A1^2*A4^2 - 8*A1^2*A4*A6 - 8*A1^2*A5^2 + 8*A1*A2^2*A3 - 32*A1*A2*A4*A5 - 16*A1*A2*A5*A6 + 8*A1*A3^3 - 8*A1*A3*A4^2 - 8*A1*A3*A5^2 + 8*A1*A3*A6^2 - 8*A1*A3*l1^2 - 4*A2^4 - 8*A2^2*A4^2 - 8*A2^2*A4*A6 - 8*A2^2*A5^2 - 16*A2*A3*A4*A5 + 4*A3^4 + 8*A3^2*A4*A6 + 8*A3^2*A6^2 - 8*A3^2*l1^2 - 4*A4^4 - 8*A4^3*A6 + 8*A4^2*A5^2 + 8*A4*A5^2*A6 + 8*A4*A6^3 - 8*A4*A6*l1^2 - 4*A5^4 + 4*A6^4 - 8*A6^2*l1^2 + 4*l1^4;
C8 =  (8*A2^4 + 16*A2^2*A5^2 + 8*A5^4)*x12^3 + (- 24*A2^4 - 48*A2^2*A5^2 - 24*A5^4)*x12^2*x22 + (24*A2^4 + 48*A2^2*A5^2 + 24*A5^4)*x12*x22^2 + (8*A1^2*A2^2 - 8*A1^2*A5^2 + 16*A1*A2^2*A3 - 32*A1*A2*A4*A5 - 32*A1*A2*A5*A6 - 16*A1*A3*A5^2 - 8*A2^4 + 8*A2^2*A3^2 - 8*A2^2*A4^2 - 16*A2^2*A4*A6 - 16*A2^2*A5^2 - 8*A2^2*A6^2 + 8*A2^2*l1^2 - 32*A2*A3*A4*A5 - 32*A2*A3*A5*A6 - 8*A3^2*A5^2 + 8*A4^2*A5^2 + 16*A4*A5^2*A6 - 8*A5^4 + 8*A5^2*A6^2 + 8*A5^2*l1^2)*x12 + (- 8*A2^4 - 16*A2^2*A5^2 - 8*A5^4)*x22^3 + (- 8*A1^2*A2^2 + 8*A1^2*A5^2 - 16*A1*A2^2*A3 + 32*A1*A2*A4*A5 + 32*A1*A2*A5*A6 + 16*A1*A3*A5^2 + 8*A2^4 - 8*A2^2*A3^2 + 8*A2^2*A4^2 + 16*A2^2*A4*A6 + 16*A2^2*A5^2 + 8*A2^2*A6^2 - 8*A2^2*l1^2 + 32*A2*A3*A4*A5 + 32*A2*A3*A5*A6 + 8*A3^2*A5^2 - 8*A4^2*A5^2 - 16*A4*A5^2*A6 + 8*A5^4 - 8*A5^2*A6^2 - 8*A5^2*l1^2)*x22;
C9 =  (A2^4 + 2*A2^2*A5^2 + A5^4)*x12^4 + (- 4*A2^4 - 8*A2^2*A5^2 - 4*A5^4)*x12^3*x22 + (6*A2^4 + 12*A2^2*A5^2 + 6*A5^4)*x12^2*x22^2 + (2*A1^2*A2^2 - 2*A1^2*A5^2 + 4*A1*A2^2*A3 - 8*A1*A2*A4*A5 - 8*A1*A2*A5*A6 - 4*A1*A3*A5^2 - 2*A2^4 + 2*A2^2*A3^2 - 2*A2^2*A4^2 - 4*A2^2*A4*A6 - 4*A2^2*A5^2 - 2*A2^2*A6^2 + 2*A2^2*l1^2 - 8*A2*A3*A4*A5 - 8*A2*A3*A5*A6 - 2*A3^2*A5^2 + 2*A4^2*A5^2 + 4*A4*A5^2*A6 - 2*A5^4 + 2*A5^2*A6^2 + 2*A5^2*l1^2)*x12^2 + (- 4*A2^4 - 8*A2^2*A5^2 - 4*A5^4)*x12*x22^3 + (- 4*A1^2*A2^2 + 4*A1^2*A5^2 - 8*A1*A2^2*A3 + 16*A1*A2*A4*A5 + 16*A1*A2*A5*A6 + 8*A1*A3*A5^2 + 4*A2^4 - 4*A2^2*A3^2 + 4*A2^2*A4^2 + 8*A2^2*A4*A6 + 8*A2^2*A5^2 + 4*A2^2*A6^2 - 4*A2^2*l1^2 + 16*A2*A3*A4*A5 + 16*A2*A3*A5*A6 + 4*A3^2*A5^2 - 4*A4^2*A5^2 - 8*A4*A5^2*A6 + 4*A5^4 - 4*A5^2*A6^2 - 4*A5^2*l1^2)*x12*x22 + (A2^4 + 2*A2^2*A5^2 + A5^4)*x22^4 + (2*A1^2*A2^2 - 2*A1^2*A5^2 + 4*A1*A2^2*A3 - 8*A1*A2*A4*A5 - 8*A1*A2*A5*A6 - 4*A1*A3*A5^2 - 2*A2^4 + 2*A2^2*A3^2 - 2*A2^2*A4^2 - 4*A2^2*A4*A6 - 4*A2^2*A5^2 - 2*A2^2*A6^2 + 2*A2^2*l1^2 - 8*A2*A3*A4*A5 - 8*A2*A3*A5*A6 - 2*A3^2*A5^2 + 2*A4^2*A5^2 + 4*A4*A5^2*A6 - 2*A5^4 + 2*A5^2*A6^2 + 2*A5^2*l1^2)*x22^2 + A1^4 + 4*A1^3*A3 - 2*A1^2*A2^2 + 6*A1^2*A3^2 + 2*A1^2*A4^2 + 4*A1^2*A4*A6 + 2*A1^2*A5^2 + 2*A1^2*A6^2 - 2*A1^2*l1^2 - 4*A1*A2^2*A3 + 8*A1*A2*A4*A5 + 8*A1*A2*A5*A6 + 4*A1*A3^3 + 4*A1*A3*A4^2 + 8*A1*A3*A4*A6 + 4*A1*A3*A5^2 + 4*A1*A3*A6^2 - 4*A1*A3*l1^2 + A2^4 - 2*A2^2*A3^2 + 2*A2^2*A4^2 + 4*A2^2*A4*A6 + 2*A2^2*A5^2 + 2*A2^2*A6^2 - 2*A2^2*l1^2 + 8*A2*A3*A4*A5 + 8*A2*A3*A5*A6 + A3^4 + 2*A3^2*A4^2 + 4*A3^2*A4*A6 + 2*A3^2*A5^2 + 2*A3^2*A6^2 - 2*A3^2*l1^2 + A4^4 + 4*A4^3*A6 - 2*A4^2*A5^2 + 6*A4^2*A6^2 - 2*A4^2*l1^2 - 4*A4*A5^2*A6 + 4*A4*A6^3 - 4*A4*A6*l1^2 + A5^4 - 2*A5^2*A6^2 - 2*A5^2*l1^2 + A6^4 - 2*A6^2*l1^2 + l1^4;

% the coefficient matrix of the 8-degree polynomials equation
CoefficientMatrix8_Ci = [C1, C2, C3, C4, C5, C6, C7, C8, C9];

% calculate the root of the 8-degree polynomials equation
%---------- x = roots()-----------
%x = roots(CoefficientMatrix8_Ci);
%x_verify = polyval(CoefficientMatrix8_Ji,x)
%%----------------------------------------------------------------------------  

%error = CoefficientMatrix8_Ci/CoefficientMatrix8_Ji

%%
% get the value of real number (+/-)
x = x(imag(x) == 0);
% x = real(x);
NumRealq13 = length (x);

% the final value of "+/-theta13" 
% the programm should judge the sign of the value
% q13all =  2 * atan(x)  - q12;

% For real elements of X, atan(X) returns values in the interval [-pi/2,pi/2]
% Here we elimate the q13all = +/-pi, namely, p = [0 0 0 0 0 0]
j =0;
for i = 1:length (x)    
    if  abs( abs(2 * atan(x(i)) - q12) - pi ) > 1e-8 
        j = j + 1;
         q13all(j) =  2 * atan(x(i))  - q12;
    else
        continue;
    end
end

%% ------------------Obtain all of the correct values and assign to q13q23-------------------------
% numbers of i and j are used to count the possible values that satisfy the
% condiation of C1z = C2z; And then, assign all the possible values to
% matrix q13q23;
tic
for Numq13 = 1:length(q13all)
    
    q13SingleValue = q13all(Numq13);
    
    %% -------------- Calculate the value of q23 (theta23) ----------------
    % (q22+q23all)*180/pi
    % sin(q12) + sin(q12+q13SingleValue) - sin(q22) - sin(q22+q23all) == 0
    %  For real elements of x in the interval [-1,1], asin(x) returns values in the interval [-pi/2,pi/2]
    % iterative method to get the optimal value
    for OnlyUsedForFolding_IterativeSolution = 1:1
        %     j = 0;
        %     delta_q13 = 0.1;
        %     while(abs(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) >= 1 && j <= 500)
        %         j = j + 1;
        %         if q12 + q13SingleValue > pi/2 || (q12 + q13SingleValue > -pi/2 && q12 + q13SingleValue <= 0)
        %             q13SingleValue = q13SingleValue + delta_q13;
        %         elseif q12 + q13SingleValue < -pi/2 || (q12 + q13SingleValue > 0 && q12 + q13SingleValue < pi/2)
        %             q13SingleValue = q13SingleValue - delta_q13;
        %         end
        %     end
    end
    %
    q23all(1) = asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
    if sin(q12) + sin(q12 + q13SingleValue) - sin(q22) <= 0
        q23all(2) =  - pi - asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
    else
        q23all(2) =    pi - asin(sin(q12) + sin(q12 + q13SingleValue) - sin(q22)) - q22;
    end
    % q23all * 180 / pi
    
    for Numq23 = 1:length(q23all)
        q23SingleValue = q23all(Numq23);
        %%-----------------------Get the output values of Moving Platform-----------------------
        %%--------------------Calculate the position of Ai Bi Ci------------------
        A1 = [0, -L1/2, 0];
        B1 = [L2 * cos(q12) * sin(q11), -L1/2 - L2 * cos(q12) * cos(q11), L2 * sin(q12)];
        C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
            L2 * (sin(q12) + sin(q12 + q13SingleValue))];
        
        A2 = [0, L1/2, 0];
        B2 = [- L2 * cos(q22) * sin(q21), L1/2 + L2 * cos(q22) * cos(q21), L2 * sin(q22)];
        C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
            L2 * (sin(q22) + sin(q22 + q23SingleValue))];
        %%------------------------------------------------------------------------
        JudgeLength_C1C2(Numq23) = norm(C1 - C2) - L1;        
       % Plot;
    end    
    norm(C1 - C2) - L1
    %Choose the column of minmum solution
    [~,col] = find(JudgeLength_C1C2 == min(JudgeLength_C1C2));
    
    %% Note:::::::::::::
    %  The value from 8-degree polynomials equation is not correct!!!!!
    % In order to get the right value, here, I use the calculated minimum q13SingleValue data to approch the correct value
       
    % Approching Algorithm is a iterative pocesss see below in
    % While loop
    if   abs(abs( q13all(Numq13) ) - pi) > 1e-8 || abs(JudgeLength_C1C2(col(1))) > 1e-8
        if abs(C1(3) - C2(3)) < 1e-8 && abs(abs( q13all(Numq13) ) - pi) > 1e-8 && ...
                abs(abs( q23all(col(1)) ) - pi) > 1e-8 && abs(JudgeLength_C1C2(col(1))) < 1e-8
            JudgeLength_C1C2_min = JudgeLength_C1C2(col(1));
        else
            %---------- Method I ----------
            for OnlyUsedForFolding_IterativeSolution = 1:1
                % We simple the one cycle points
                ii = 0;
                for i_angle = 0:1*pi/180:pi
                    q13SingleValue = i_angle;
                    %
                    sin_value = sin(q12) + sin(q12 + q13SingleValue) - sin(q22);
                    zC1 = sin(q12) + sin(q12 + q13SingleValue);
                    %
                    if sin_value > 1 || zC1 < 0
                        continue;
                    end
                    ii = ii + 1;
                    q13SingleValue_mat(ii) = i_angle;
                    %
                    q23all(1) = asin(sin_value) - q22;
                    if sin_value <= 0
                        q23all(2) =  - pi - asin(sin_value) - q22;
                    else
                        q23all(2) =    pi - asin(sin_value) - q22;
                    end
                    % Calculate q23SingleValue = -pi ~ pi
                    for i_q23all = 1:2
                        q23SingleValue = q23all(i_q23all);
                        C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                            L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                        C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                            L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                        JudgeLength_C1C2_Solution(i_q23all,ii) = norm(C1 - C2) - L1;
                        q23SingleValue_mat(i_q23all,ii) = q23all(i_q23all);
                        if i_q23all == 1
                            C11_mat(ii,:) = C1;
                            C21_mat(ii,:) = C2;
                        else
                            C12_mat(ii,:) = C1;
                            C22_mat(ii,:) = C2;
                        end
                    end
                    
                end
                plot3(C11_mat(:,1), C11_mat(:,2), C11_mat(:,3),'k-','linewidth',1); hold on;
                plot3(C21_mat(:,1), C21_mat(:,2), C21_mat(:,3),'k-','linewidth',1); hold on;
                plot3(C12_mat(:,1), C12_mat(:,2), C12_mat(:,3),'k-.','linewidth',1); hold on;
                plot3(C22_mat(:,1), C22_mat(:,2), C22_mat(:,3),'k-.','linewidth',1); hold on;                
                
                for i_q23all = 1:2
                    B = JudgeLength_C1C2_Solution(i_q23all,:);
                    col_B_positive = find(B>0);
                    col_B_zero = find(B <= 1e-8);
                    if isempty(col_B_zero) ~= 1 && min(col_B_zero) < length(B)
                        Changepoint_positive_min = min(col_B_zero);
                        i_q23all_selected = i_q23all;
                        JudgeLength_C1C2_min = 0;
                    else
                        j = 0;
                        Changepoint_postive = [];
                        for i = 1:length(col_B_positive)
                            if i == 1 && col_B_positive(1)>1
                                j = j + 1;
                                Changepoint_postive(j) = col_B_positive(i);
                            elseif i < length(col_B_positive) && col_B_positive(i+1) - col_B_positive(i) > 1
                                j = j + 1;
                                Changepoint_postive(j) = col_B_positive(i);
                                j = j + 1;
                                Changepoint_postive(j) = col_B_positive(i+1);
                            elseif i == length(col_B_positive) && col_B_positive(length(col_B_positive))<length(B)
                                j = j + 1;
                                Changepoint_postive(j) = col_B_positive(i);
                            end
                            Changepoint_positive_min = min(Changepoint_postive);
                        end
                        
                        if isempty(Changepoint_postive) == 1
                            JudgeLength_C1C2_min = [];
                            Changepoint_positive_min = [];
                            continue
                        else
                            JudgeLength_C1C2_min = min(B(Changepoint_postive));
                            i_q23all_selected = i_q23all;
                        end
                    end
                    
                    q13SingleValue = q13SingleValue_mat(Changepoint_positive_min);
                    q23SingleValue = q23SingleValue_mat(i_q23all_selected,Changepoint_positive_min);
                    C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                        L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                    C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                        L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                    
                    plot3(C12_mat(Changepoint_positive_min,1), C12_mat(Changepoint_positive_min,2), C12_mat(Changepoint_positive_min,3),'r.','markersize',10); hold on;
                    
                    %---------------------- Iterative Start --------------------
                    delta_q13 = 0.05;
                    q13SingleValue = q13SingleValue - delta_q13;                     
                    % norm(C1 - C2) - L1
                    SignChange_1 = 0;
                    SignChange_2 = 0;
                    k = 0;
                    % iterative method to get the optimal value
                    while(abs(norm(C1 - C2) - L1) > 1e-8 && k <= 500)
                        k = k + 1;
                        % Here, we must make sure 'sin_value belongs to [-1 1]'
                        sin_value = sin(q12) + sin(q12 + q13SingleValue) - sin(q22);
                        %
                        q23all(1) = asin(sin_value) - q22;
                        if sin_value <= 0
                            q23all(2) =  - pi - asin(sin_value) - q22;
                        else
                            q23all(2) =    pi - asin(sin_value) - q22;
                        end
                        
                        % Choose the correct q23all
                        q23SingleValue = q23all(i_q23all_selected);
                        C1 = [L2 * (cos(q12) + cos(q12 + q13SingleValue)) * sin(q11), -L1/2 - L2 * (cos(q12) + cos(q12 + q13SingleValue)) * cos(q11), ...
                            L2 * (sin(q12) + sin(q12 + q13SingleValue))];
                        C2 = [- L2 * (cos(q22) + cos(q22 + q23SingleValue)) * sin(q21), L1/2 + L2 * (cos(q22) + cos(q22 + q23SingleValue)) * cos(q21),...
                            L2 * (sin(q22) + sin(q22 + q23SingleValue))];
                        
                        JudgeLength_C1C2_min = norm(C1 - C2) - L1;
                        %JudgeLength_C1C2_min
                        if JudgeLength_C1C2_min > 0
                            if SignChange_1 == 0
                                if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min < 0
                                    delta_q13 = 0.5 * delta_q13;
                                elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min >= 0
                                    delta_q13 = - 0.5 * delta_q13;
                                end
                            end
                            SignChange_1 = SignChange_1 + 1;
                            SignChange_2 = 0;
                        elseif JudgeLength_C1C2_min < 0
                            if SignChange_2 == 0   
                                if(norm(C1 - C2) - L1) - JudgeLength_C1C2_min > 0
                                    delta_q13 = 0.5 * delta_q13;
                                elseif (norm(C1 - C2) - L1) - JudgeLength_C1C2_min <= 0
                                    delta_q13 = - 0.5 * delta_q13;
                                end
                            end
                            SignChange_1 = 0;
                            SignChange_2 = SignChange_2 + 1;
                        end
                        
                        q13SingleValue = q13SingleValue - delta_q13;
                        %hold off
                        %Plot;
                    end
                    
                    %---------------------- Iterative end --------------------
                end
            end
        end
        % assign the values to q13q23
        q13 = q13SingleValue;
        q23 = q23SingleValue;
        
        % Calculate the center point of moving platform
        p(1:3) = (C1 + C2) / 2;
        C1C2 = C2 - C1;
        yaxis = [0, 1, 0];
        % Judge the direction of the rotation around Z-axis
        if C1(1) < C2(1)
            gamma = -acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
        else
            gamma = acos((C1C2 * yaxis')/(norm(C1C2)* norm(yaxis)));
        end
        p(4:6) = [0, 0, gamma];
        ABC = [A1, B1, C1, A2, B2, C2];
        
        %%-------------------------q14q15 and q24q25------------------------------
        % Calculate the angles of q14q15 and q24q25
        q14 = q12 + q13 - pi/2;
        q15 = q11;
        q24 = q22 + q23 - pi/2;
        q25 = q21;
        %%------------------------------------------------------------------------
        %%-------------------------q11-q15 and q21-q25------------------------------
        q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25]; 
        if isempty(JudgeLength_C1C2_min) == 1 || JudgeLength_C1C2_min < 1e-8 
            if isempty(JudgeLength_C1C2_min) == 1 || abs( abs(q13) - pi) < 1e-8
                display('Only Zero Position exist, Calculation process is stopped!');
            end
            break
        else
            continue
        end
    elseif  abs(abs( q13all(Numq13) ) - pi) < 1e-8 && abs(JudgeLength_C1C2(col(1))) < 1e-8
        display('Zero Position, Calculation process needs to check!');
        p = [0 0 0 0 0 0];
        gamma = 0;
        q13 = pi;
        q23 = pi;
        ABC = [A1, B1, C1, A2, B2, C2];
        q14 = q12 + q13 - pi/2;
        q15 = q11;
        q24 = q22 + q23 - pi/2;
        q25 = q21;
        q1q2 = [q11, q12, q13, q14, q15, q21, q22, q23, q24, q25];  
    else
        display('No solution for this input, Calculation process is stopped');
        p = [];
        ABC = [];
        q1q2 = [0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];
    end        

end
toc

norm(C1 - C2) - L1
%k
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
 
 grid on;
 axis equal;
 xlabel('x');
 ylabel('y');
 zlabel('z');
 
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
axis equal;
%-----------------------------------------------------------


