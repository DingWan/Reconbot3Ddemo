function [c, ceq] = Z_value_equal(x, q12, q22) 
ceq = sin(q12) + sin(q12 + x(1)) - sin(q22) - sin(q22 + x(2));
c = [];
end