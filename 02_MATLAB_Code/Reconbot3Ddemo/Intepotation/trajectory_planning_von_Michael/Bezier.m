% % % --------------------------------
% % % Author: begtostudy
% % % Email : begtostudy@gmail.com
% % % --------------------------------
function [Q,Qs,Qss]=Bezier(P,t)
% Bezier interpolation for given points.
 
for k=1:length(t)
    Q(:,k)=[0 0 0]';
    for j=1:size(P,2)
        Q(:,k)=Q(:,k)+P(:,j)*Bernstein(size(P,2)-1,j-1,t(k));
    end
end

for k=1:length(t)
    Qs(:,k)=[0 0 0]';
    for j=1:size(P,2)-1
        Qs(:,k)=Qs(:,k)+(P(:,j+1)-P(:,j))*Bernstein(size(P,2)-2,j-1,t(k));
    end
end
Qs=Qs*(size(P,2)-1);

for k=1:length(t)
    Qss(:,k)=[0 0 0]';
    for j=1:size(P,2)-2
        Qss(:,k)=Qss(:,k)+(P(:,j+2)-2*P(:,j+1)+P(:,j))*Bernstein(size(P,2)-3,j-1,t(k));
    end
end
Qss=Qss*(size(P,2)-1)*(size(P,2)-2);
end
 
function B=Bernstein(n,j,t)
    B=factorial(n)/(factorial(j)*factorial(n-j))*(t^j)*(1-t)^(n-j);
end