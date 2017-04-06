function [ a0,a1,a2,a3,a4,a5,tau ] = Polynom5( p,T_ges )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Berechnung der Zeitintervalle
s_ges = 0;
s = zeros(1,size(p,2)-1);
for i = 1 : size(p,2)-1
    s(i) = norm(p(:,i+1)-p(:,i));
    s_ges = s_ges+s(i);
end
tau = T_ges/s_ges*s;

% Linearinterpolation
l = zeros(size(p,1),size(p,2)-1);
for i = 1 : size(p,2)-1
    l(:,i) = (p(:,i+1)-p(:,i))/tau(i);
end

% Steigung und Krümmung der Zwischenpunkten
ps = zeros(size(p,1),size(p,2));
pss = zeros(size(p,1),size(p,2));
for i = 1 : size(p,2)-2
    ps(:,i+1) = (tau(i+1)*l(:,i)+tau(i)*l(:,i+1))/(tau(i+1)+tau(i));
    pss(:,i+1) = zeros(3,1);
%     pss(:,i+1) = (l(:,i+1)-l(:,i))/(tau(i+1)+tau(i))*2;
end

% Steigung und Krümmung des Anfangs-/Endpunktes
ps(:,1) = l(:,1)-tau(1)/3*pss(:,2);
ps(:,size(p,2)) = l(:,size(p,2)-1)-tau(size(p,2)-1)/3*pss(:,size(p,2)-1);
pss(:,1) = zeros(size(p,1),1);
pss(:,size(p,2)) = zeros(size(p,1),1);

a0 = zeros(size(p,1),size(p,2)-1);
a1 = zeros(size(p,1),size(p,2)-1);
a2 = zeros(size(p,1),size(p,2)-1);
a3 = zeros(size(p,1),size(p,2)-1);
a4 = zeros(size(p,1),size(p,2)-1);
a5 = zeros(size(p,1),size(p,2)-1);
for i = 1 : size(p,2)-1
    a0(:,i) = p(:,i);
    a1(:,i) = ps(:,i);
    a2(:,i) = 1/2*pss(:,i);
    a3(:,i) = (1/(2*tau(i)^3)) * (20 * (p(:,i+1) - p(:,i)) - (8 * ps(:,i+1)+ 12*ps(:,i) )*tau(i) - (3 * pss(:,i+1) - pss(:,i) )*tau(i)^2 );
    a4(:,i) = (1/(2*tau(i)^4)) * (30 * (p(:,i) - p(:,i+1)) + (14 * ps(:,i+1)+ 16*ps(:,i) )*tau(i) + (3 * pss(:,i+1) - 2*pss(:,i) )*tau(i)^2 );
    a5(:,i) = (1/(2*tau(i)^5)) * (12 * (p(:,i+1) - p(:,i)) - 6*(ps(:,i+1)+ ps(:,i) )*tau(i) - (pss(:,i+1) - pss(:,i) )*tau(i)^2 );
end

end

