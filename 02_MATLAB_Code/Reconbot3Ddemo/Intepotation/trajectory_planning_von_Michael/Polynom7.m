function [ a0,a1,a2,a3,a4,a5,a6,a7,tau ] = Polynom7( p,T_ges )
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
psss = zeros(size(p,1),size(p,2));
for i = 1 : size(p,2)-2
    ps(:,i+1) = (tau(i+1)*l(:,i)+tau(i)*l(:,i+1))/(tau(i+1)+tau(i));
%     pss(:,i+1) = zeros(3,1);
    pss(:,i+1) = (l(:,i+1)-l(:,i))/(tau(i+1)+tau(i))*2;
    psss(:,i+1) = zeros(3,1);
end

% Steigung und Krümmung des Anfangs-/Endpunktes
ps(:,1) = zeros(size(p,1),1);
ps(:,size(p,2)) = zeros(size(p,1),1);
pss(:,1) = zeros(size(p,1),1);
pss(:,size(p,2)) = zeros(size(p,1),1);
psss(:,1) = zeros(size(p,1),1);
psss(:,size(p,2)) = zeros(size(p,1),1);

a0 = zeros(size(p,1),size(p,2)-1);
a1 = zeros(size(p,1),size(p,2)-1);
a2 = zeros(size(p,1),size(p,2)-1);
a3 = zeros(size(p,1),size(p,2)-1);
a4 = zeros(size(p,1),size(p,2)-1);
a5 = zeros(size(p,1),size(p,2)-1);
a6 = zeros(size(p,1),size(p,2)-1);
a7 = zeros(size(p,1),size(p,2)-1);
for i = 1 : size(p,2)-1
    a0(:,i) = p(:,i);
    a1(:,i) = ps(:,i);
    a2(:,i) = 1/2*pss(:,i);
    a3(:,i) = 1/6*psss(:,i);
    a4(:,i) = (1/(6*tau(i)^4)) * ( 210 * (p(:,i+1)-p(:,i)) - tau(i) * ((30*pss(:,i)-15*pss(:,i+1))*tau(i) + (4*psss(:,i)+1*psss(:,i+1))*tau(i)^2 + 120*ps(:,i)+90*ps(:,i+1)));
    a5(:,i) = (1/(2*tau(i)^5)) * (-168 * (p(:,i+1)-p(:,i)) + tau(i) * ((20*pss(:,i)-14*pss(:,i+1))*tau(i) + (2*psss(:,i)+1*psss(:,i+1))*tau(i)^2 +  90*ps(:,i)+78*ps(:,i+1)));
    a6(:,i) = (1/(6*tau(i)^6)) * ( 420 * (p(:,i+1)-p(:,i)) - tau(i) * ((45*pss(:,i)-39*pss(:,i+1))*tau(i) + (4*psss(:,i)+3*psss(:,i+1))*tau(i)^2 + 216*ps(:,i)+204*ps(:,i+1)));
    a7(:,i) = (1/(6*tau(i)^7)) * (-120 * (p(:,i+1)-p(:,i)) + tau(i) * ((12*pss(:,i)-12*pss(:,i+1))*tau(i) + (1*psss(:,i)+1*psss(:,i+1))*tau(i)^2 +  60*ps(:,i)+60*ps(:,i+1)));
end

end

