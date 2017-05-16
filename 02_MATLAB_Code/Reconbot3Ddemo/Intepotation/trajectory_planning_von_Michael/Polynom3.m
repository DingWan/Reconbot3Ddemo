function [ a0,a1,a2,a3,tau ] = Polynom3( p,T_ges )
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
for i = 1 : size(p,2)-1
    a0(:,i) = p(:,i);
    a1(:,i) = ps(:,i);
    a2(:,i) = 3/tau(i)^2 * (p(:,i+1) - p(:,i)) - 1/tau(i) * (ps(:,i+1) + 2*ps(:,i));
    a3(:,i) = -2/tau(i)^3 * (p(:,i+1) - p(:,i)) + 1/tau(i)^2 * (ps(:,i+1) + ps(:,i));
end

end

