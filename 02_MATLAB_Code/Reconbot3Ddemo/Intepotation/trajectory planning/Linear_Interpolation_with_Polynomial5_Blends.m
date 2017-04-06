function [ a0,a1,a2,a3,a4,a5,p0,p1,p2,p3,p4,p5,t0,t5,q,q1,q2 ] = Linear_Interpolation_with_Polynomial5_Blends( p,r )
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
    t5(:,i) = (q2(:,i) - q(:,i))/r(i);
end

% Berechnung der Start- und Endpunkte
for i = 2 : size(p,2)-1
    p0(:,i) = q1(:,i);
    p5(:,i) = q2(:,i);
    a(i) = 256 - 49*norm(t0(:,i)+t5(:,i))^2;
    b(i) = 420*(p5(:,i)-p0(:,i))'*(t0(:,i)+t5(:,i));
    c(i) = -900*norm(p5(:,i)-p0(:,i))^2;
    alpha(i) = max(roots([a(i) b(i) c(i)]));
    p1(:,i) = p0(:,i) + 1/5*alpha(i)*t0(:,i);
    p2(:,i) = 2*p1(:,i) - p0(:,i);
    p4(:,i) = p5(:,i) - 1/5*alpha(i)*t5(:,i);
    p3(:,i) = 2*p4(:,i) - p5(:,i);
end
p0(:,1) = q1(:,1);
p1(:,1) = q1(:,1);
p2(:,1) = q1(:,1);
p3(:,1) = q1(:,1);
p4(:,1) = q1(:,1);
p5(:,1) = q1(:,1);
p0(:,size(p,2)) = q1(:,size(p,2));
p1(:,size(p,2)) = q1(:,size(p,2));
p2(:,size(p,2)) = q1(:,size(p,2));
p3(:,size(p,2)) = q1(:,size(p,2));
p4(:,size(p,2)) = q1(:,size(p,2));
p5(:,size(p,2)) = q1(:,size(p,2));

% Berechnung der Polynomkoeffizienten
for i = 1 : size(p,2)-1
    a0(:,2*i-1) = p5(:,i);
    a1(:,2*i-1) = p0(:,i+1) - p5(:,i);
    a2(:,2*i-1) = zeros(3,1);
    a3(:,2*i-1) = zeros(3,1);
    a4(:,2*i-1) = zeros(3,1);
    a5(:,2*i-1) = zeros(3,1);
end
for i = 2 : size(p,2)-1
    a0(:,2*i-2) = p0(:,i);
    a1(:,2*i-2) = -5*p0(:,i) + 5*p1(:,i);
    a2(:,2*i-2) = 10*p0(:,i) - 20*p1(:,i) + 10*p2(:,i);
    a3(:,2*i-2) = -10*p0(:,i) + 30*p1(:,i) - 30*p2(:,i) + 10*p3(:,i);
    a4(:,2*i-2) = 5*p0(:,i) - 20*p1(:,i) + 30*p2(:,i) - 20*p3(:,i) + 5*p4(:,i);
    a5(:,2*i-2) = -p0(:,i) + 5*p1(:,i) - 10*p2(:,i) + 10*p3(:,i) - 5*p4(:,i) + p5(:,i);
end

end

