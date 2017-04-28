function [c, ceq] = Z_value_equal(x) 
q11q12q21q22 = [1.2*pi/4, 1*pi/2, 1*pi/4, 0.6*pi/2];
q11 = q11q12q21q22(1);
q12 = q11q12q21q22(2);
q21 = q11q12q21q22(3);
q22 = q11q12q21q22(4);
ceq = sin(q12) + sin(q12 + x(1)) - sin(q22) - sin(q22 + x(2));
c = [];
end