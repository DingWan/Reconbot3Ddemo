function p=SpaceArcGenerator(p1,p2,p3,n)
    a=p1;
    b=p2;
    c=p3;
    jz=[a;b;c];
    Rr=CircleCenter(jz);
    R=halfcolumn(a,Rr);
    A=[R 0 0];
    SP=circlepath(a,b,c,A,Rr,R,n);
     xp=SP(1,:);
     yp=SP(2,:);
     zp=SP(3,:);
    xz=rotmartix(a,b,c,Rr);
    e=xz'*[xp;yp;zp];
    lie=size(e,2);
    dw=ones(3,lie);
    xj=[Rr(1)*dw(1,:);Rr(2)*dw(2,:);Rr(3)*dw(3,:)];
    p=e+xj;
end

function p=CircleCenter(M)
 
    x1=M(1,1);
    y1=M(1,2);
    z1=M(1,3);
    x2=M(2,1);
    y2=M(2,2);
    z2=M(2,3);
    x3=M(3,1);
    y3=M(3,2);
    z3=M(3,3);
    A1 = y1*z2-y1*z3-z1*y2+z1*y3+y2*z3-y3*z2;
    B1 = -x1*z2+x1*z3+z1*x2-z1*x3-x2*z3+x3*z2;
    C1 = x1*y2-x1*y3-y1*x2+y1*x3+x2*y3-x3*y2;
    D1 = -x1*y2*z3+x1*y3*z2+x2*y1*z3-x3*y1*z2-x2*y3*z1+x3*y2*z1;
    A2 = 2*(x2-x1);
    B2 = 2*(y2-y1);
    C2 = 2*(z2-z1);
    D2 = x1^2+y1^2+z1^2-x2^2-y2^2-z2^2;
    A3 = 2*(x3-x1);
    B3 = 2*(y3-y1);
    C3 = 2*(z3-z1);
    D3 = x1^2+y1^2+z1^2-x3^2-y3^2-z3^2;
    RO =[A1 B1 C1;A2 B2 C2;A3 B3 C3]\[-D1;-D2;-D3];
    x0 =RO(1);
    y0 =RO(2);
    z0 =RO(3);
     
    p=[x0 y0 z0];
%    plot3(x0,y0,z0,'*');grid on;
end

function R1=halfcolumn(p1,r)
    x1=p1(1);
    y1=p1(2);
    z1=p1(3);
    x0=r(1);
    y0=r(2);
    z0=r(3);
     
    R1= sqrtm((x1-x0)^2+(y1-y0)^2+(z1-z0)^2);
end

function p=circlepath(b1,b2,b3,S,Y,bj,n)
 
    P=rotmartix(b1,b2,b3,Y);
    x3=b3(1);
    y3=b3(2);
    z3=b3(3);
 
    cp=P*[x3-Y(1);y3-Y(2);z3-Y(3)];
 
    F=1;
    T=0.03;
    l=F*T;
    d=sqrtm((cp(1)-S(1))^2+(cp(2)-S(2))^2);
    
    alfa=2*asin(d/(2*bj));
    Op=[0 0 0];
    if cp(2)>0;
        beta=alfa;
    else cp(2)<0;
        beta=2*pi-alfa;
    end
    %s=beta*bj;
    %n=50;
    for i=0:n    
            x(i+1)=Op(1)+bj*cos((beta*i)/n);%根据三个点确定圆弧，从圆弧的起始点画到终止点
            y(i+1)=Op(2)+bj*sin((beta*i)/n);
            z(i+1)=0;
    end
    p=[x;y;z];
end

% 求叉积 ab X bc = ui+vj+wk
%n为平面ABC的法向量
% 等于matlab函数 cross(ab,bc) 
function n =rotmartix(p1,p2,p3,O)
 
u = (p2(2)-p1(2))*(p3(3)-p2(3))-(p2(3)-p1(3))*(p3(2)-p2(2));
v = (p2(3)-p1(3))*(p3(1)-p2(1))-(p2(1)-p1(1))*(p3(3)-p2(3));
w = (p2(1)-p1(1))*(p3(2)-p2(2))-(p2(2)-p1(2))*(p3(1)-p2(1));
 
k=[u v w]/sqrtm(u^2+v^2+w^2);
a=p1;
OA=(a-O);
i=(a-O)/sqrtm(dot(OA,OA));
j=cross(k,i);
n=[i;j;k];
end