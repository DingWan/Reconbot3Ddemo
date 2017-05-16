function [ a0,a1,a2,a3,a4,a5,tau,pn ] = Linear_Interpolation_with_Polynomial_Blends( p,T_ges,r )
% [ a0,a1,a2,a3,a4,a5,tau,pn ] = Blending_Polynom5( p,T_ges,r )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Berechnung der Stützstellen
pn = [p(:,1),zeros(size(p,1),2*(size(p,2)-2)),p(:,size(p,2))];
for i = 2 : size(p,2)-1
    pn(:,2*i-2) = p(:,i)+(p(:,i-1)-p(:,i))*r(i)/norm(p(:,i-1)-p(:,i));
    pn(:,2*i-1) = p(:,i)+(p(:,i+1)-p(:,i))*r(i)/norm(p(:,i+1)-p(:,i));
end

% Berechnung der Zeitintervalle
s_ges = 0;
s = zeros(1,size(pn,2)-1);
for i = 1 : size(pn,2)-1
    s(i) = norm(pn(:,i+1)-pn(:,i));
    s_ges = s_ges+s(i);
end
tau = T_ges/s_ges*s;

% Steigung und Krümmung der Stützstellen
ps = zeros(size(pn,1),size(pn,2));
pss = zeros(size(pn,1),size(pn,2));
for i = 1 : size(p,2)-1
    ps(:,2*i-1) = (p(:,i+1)-p(:,i))/norm(p(:,i+1)-p(:,i));
    ps(:,2*i) = (p(:,i+1)-p(:,i))/norm(p(:,i+1)-p(:,i));
    pss(:,2*i-1) = zeros(3,1);
    pss(:,2*i-1) = zeros(3,1);
end

% Berechnung der Polynomkoeffizienten
a0 = zeros(size(pn,1),size(pn,2)-2);
a1 = zeros(size(pn,1),size(pn,2)-2);
a2 = zeros(size(pn,1),size(pn,2)-2);
a3 = zeros(size(pn,1),size(pn,2)-2);
a4 = zeros(size(pn,1),size(pn,2)-2);
a5 = zeros(size(pn,1),size(pn,2)-2);
for i = 1 : size(pn,2)-1
        a0(:,i) = pn(:,i);
        a1(:,i) = ps(:,i);
        a2(:,i) = 1/2*pss(:,i);
        a3(:,i) = (1/(2*r((i-rem(i,2))/2+1)^3)) * (20 * (pn(:,i+1) - pn(:,i)) - (8 * ps(:,i+1)+ 12*ps(:,i) )*r((i-rem(i,2))/2+1) - (3 * pss(:,i+1) - pss(:,i) )*r((i-rem(i,2))/2+1)^2 );
        a4(:,i) = (1/(2*r((i-rem(i,2))/2+1)^4)) * (30 * (pn(:,i) - pn(:,i+1)) + (14 * ps(:,i+1)+ 16*ps(:,i) )*r((i-rem(i,2))/2+1) + (3 * pss(:,i+1) - 2*pss(:,i) )*r((i-rem(i,2))/2+1)^2 );
        a5(:,i) = (1/(2*r((i-rem(i,2))/2+1)^5)) * (12 * (pn(:,i+1) - pn(:,i)) - 6*(ps(:,i+1)+ ps(:,i) )*r((i-rem(i,2))/2+1) - (pss(:,i+1) - pss(:,i) )*r((i-rem(i,2))/2+1)^2 );
end

end

