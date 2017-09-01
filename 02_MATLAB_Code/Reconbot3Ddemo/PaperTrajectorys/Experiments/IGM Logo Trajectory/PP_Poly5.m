function [ y,ys,yss ] = PP_Poly5( x,coeff )
y=[x.^5;    x.^4;       x.^3;       x.^2;   x;  ones(size(x))].'*coeff;
ys=[5*x.^4;  4*x.^3;     3*x.^2;     2*x;    ones(size(x));  zeros(size(x))].'*coeff;
yss=[20*x.^3; 12*x.^2;    6*x;        2*ones(size(x));   zeros(size(x));  zeros(size(x))].'*coeff;
end

