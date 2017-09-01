function [ a0,a1,a2,a3,a4,p0,p1,p2,p3,p4,t0,t4,q,q1,q2 ] = Linear_Interpolation_with_Polynomial4_Blends( p,r )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

% Berechnung der Stützstellen
q  = zeros(size(p,1),size(p,2));
q1 = zeros(size(p,1),size(p,2));
q2 = zeros(size(p,1),size(p,2));
for i = 2 : size(p,2)-1
    q(:,i)  = p(:,i);
    q1(:,i) = p(:,i)+(p(:,i-1)-p(:,i))*r(i)/norm(p(:,i-1)-p(:,i));
    q2(:,i) = p(:,i)+(p(:,i+1)-p(:,i))*r(i)/norm(p(:,i+1)-p(:,i));
end
q1(:,1) = p(:,1);
q2(:,1) = p(:,1);
q1(:,size(p,2)) = p(:,size(p,2));
q2(:,size(p,2)) = p(:,size(p,2));

% Berechnung der Tangentenvektoren
for i = 2 : size(p,2)-1
    t0(:,i) = (q(:,i) - q1(:,i))/r(i);
    t4(:,i) = (q2(:,i) - q(:,i))/r(i);
end

% Berechnung der Start- und Endpunkte
for i = 2 : size(p,2)-1
    p0(:,i) = q1(:,i);
    p2(:,i) = q(:,i);
    p4(:,i) = q2(:,i);
    a(i) = 4 - 1/4*norm(t0(:,i)+t4(:,i))^2;
    b(i) = 3*(p4(:,i)-p0(:,i))'*(t0(:,i)+t4(:,i));
    c(i) = -9*norm(p4(:,i)-p0(:,i))^2;
    alpha(i) = max(roots([a(i) b(i) c(i)]));
    p1(:,i) = q1(:,i) + 1/4*alpha(i)*t0(:,i);
    p3(:,i) = q2(:,i) - 1/4*alpha(i)*t4(:,i);
end
p0(:,1) = q1(:,1);
p1(:,1) = q1(:,1);
p2(:,1) = q1(:,1);
p3(:,1) = q1(:,1);
p4(:,1) = q1(:,1);
p0(:,size(p,2)) = q1(:,size(p,2));
p1(:,size(p,2)) = q1(:,size(p,2));
p2(:,size(p,2)) = q1(:,size(p,2));
p3(:,size(p,2)) = q1(:,size(p,2));
p4(:,size(p,2)) = q1(:,size(p,2));

% Berechnung der Polynomkoeffizienten
for i = 1 : size(p,2)-1
    a0(:,2*i-1) = p4(:,i);
    a1(:,2*i-1) = p0(:,i+1) - p4(:,i);
    a2(:,2*i-1) = zeros(3,1);
    a3(:,2*i-1) = zeros(3,1);
    a4(:,2*i-1) = zeros(3,1);
end
for i = 2 : size(p,2)-1
    a0(:,2*i-2) = p0(:,i);
    a1(:,2*i-2) = -4*p0(:,i) + 4*p1(:,i);
    a2(:,2*i-2) = 6*p0(:,i) - 12*p1(:,i) + 6*p2(:,i);
    a3(:,2*i-2) = -4*p0(:,i) + 12*p1(:,i) - 12*p2(:,i) + 4*p3(:,i);
    a4(:,2*i-2) = p0(:,i) - 4*p1(:,i) + 6*p2(:,i) - 4*p3(:,i) + p4(:,i);
end

end

