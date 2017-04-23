clc
clear all
close all
tic

% Position im kartesischen Raum
% p = [ -500 , -150 , 150 , 500
%      750 ,800 , 700 , 750
%       100   , 50 ,  -50 ,  150];
% % p = [0 ,  2 ,  8 , 11 , 14 , 14
% %      1 , 10 ,  0 , 14 ,  1 ,  2
% %      0 ,  2 ,  5 , 14 , 13 , 10];
p = [0,  14
     1,  8
     0,   10];
% p = [0 , 1 , 2 , 4 , 5 , 6
%      0 , 2 , 3 , 3 , 2 , 0
%      0 , 1 , 0 , 0 , 2 , 2];
% p = [0 , 0 , 0.6 , 0.6 , 0.6 , 0.6 , 0 , 0 , 0
%     0 , 0.4 , 0.4 , 0 , 0 , 0.4 , 0.4 , 0 , 0
%     0 , 0 , 0 , 0 , 0.4 , 0.4 , 0.4 , 0.4 , 0];

% Zeitbedingung
T_ges = 4; % [sec]

% Verschnitradius
% r = [1 , 200 , 200 , 1];
r = [1 , 3 , 3 , 5 , 2 , 1];
% r = 0.5*[1 , 1 , 1 , 1 , 1 , 1];
% r = 0.05*[1 1 1 1 1 1 1 1 1];

% Verfahren
% Verfahren = 'Polynom3';
Verfahren = 'Polynom5';
% Verfahren = 'Polynom7';
% Verfahren = 'Bezier_Curve';
% Verfahren = 'Linear_Interpolation_with_Polynomial_Blends';
% Verfahren = 'Linear_Interpolation_with_Polynomial4_Blends';
% Verfahren = 'Linear_Interpolation_with_Polynomial5_Blends';
% Verfahren = 'Linear_Interpolation_with_Bezier_Blends';

switch Verfahren
    
    case 'Polynom3'
        
        [ a0,a1,a2,a3,tau ] = Polynom3( p,T_ges );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(p,2)-1
            t = linspace(0,tau(i),100);
            px = [a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
        end
        
        % Position, Geschwindigkeit, Beschleunigung am Beispiel der x-Richtung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        for i = 1 : size(p,2)-1
            j = 1;
            t = linspace(0,tau(i),100);
            px = [a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t);
            a = polyval(pxdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'blue');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 2;
            t = linspace(0,tau(i),100);
            py = [a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pyd = polyder(py);
            pydd = polyder(pyd);
            s = polyval(py,t);
            v = polyval(pyd,t);
            a = polyval(pydd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'red');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'red');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'red');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 3;
            t = linspace(0,tau(i),100);
            pz = [a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pzd = polyder(pz);
            pzdd = polyder(pzd);
            s = polyval(pz,t);
            v = polyval(pzd,t);
            a = polyval(pzdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'green');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'green');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'green');
            hold on
        end
        
        subplot(3,1,1);
        xlabel('t','FontSize',16)
        ylabel('s','FontSize',16)
        subplot(3,1,2);
        xlabel('t','FontSize',16)
        ylabel('v','FontSize',16)
        subplot(3,1,3);
        xlabel('t','FontSize',16)
        ylabel('a','FontSize',16)
        
    case 'Polynom5' % Bahninterpolation Polynom 5. Ordnung
        
        [ a0,a1,a2,a3,a4,a5,tau ] = Polynom5( p,T_ges );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(p,2)-1
            t = linspace(0,tau(i),100);
            px = [a5(1,i),a4(1,i),a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a5(2,i),a4(2,i),a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a5(3,i),a4(3,i),a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
        end
        
        % Position, Geschwindigkeit, Beschleunigung am Beispiel der x-Richtung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        for i = 1 : size(p,2)-1
            j = 1;
            t = linspace(0,tau(i),100);
            px = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t);
            a = polyval(pxdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'blue');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 2;
            t = linspace(0,tau(i),100);
            py = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pyd = polyder(py);
            pydd = polyder(pyd);
            s = polyval(py,t);
            v = polyval(pyd,t);
            a = polyval(pydd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'red');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'red');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'red');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 3;
            t = linspace(0,tau(i),100);
            pz = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pzd = polyder(pz);
            pzdd = polyder(pzd);
            s = polyval(pz,t);
            v = polyval(pzd,t);
            a = polyval(pzdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'green');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'green');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'green');
            hold on
        end
        
        subplot(3,1,1);
        xlabel('t','FontSize',16)
        ylabel('s','FontSize',16)
        subplot(3,1,2);
        xlabel('t','FontSize',16)
        ylabel('v','FontSize',16)
        subplot(3,1,3);
        xlabel('t','FontSize',16)
        ylabel('a','FontSize',16)
        
    case 'Polynom7' % Bahninterpolation Polynom 7. Ordnung
        
        [ a0,a1,a2,a3,a4,a5,a6,a7,tau ] = Polynom7( p,T_ges );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(p,2)-1
            t = linspace(0,tau(i),100);
            px = [a7(1,i),a6(1,i),a5(1,i),a4(1,i),a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a7(2,i),a6(2,i),a5(2,i),a4(2,i),a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a7(3,i),a6(3,i),a5(3,i),a4(3,i),a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
        end
        
        % Position, Geschwindigkeit, Beschleunigung am Beispiel der x-Richtung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        for i = 1 : size(p,2)-1
            j = 1;
            t = linspace(0,tau(i),100);
            px = [a7(j,i),a6(j,i),a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t);
            a = polyval(pxdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'blue');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 2;
            t = linspace(0,tau(i),100);
            py = [a7(j,i),a6(j,i),a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pyd = polyder(py);
            pydd = polyder(pyd);
            s = polyval(py,t);
            v = polyval(pyd,t);
            a = polyval(pydd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'red');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'red');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'red');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 3;
            t = linspace(0,tau(i),100);
            pz = [a7(j,i),a6(j,i),a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pzd = polyder(pz);
            pzdd = polyder(pzd);
            s = polyval(pz,t);
            v = polyval(pzd,t);
            a = polyval(pzdd,t);
            
            subplot(3,1,1);
            plot(t+sum(tau(1:i-1)),s,'green');
            hold on
            
            subplot(3,1,2);
            plot(t+sum(tau(1:i-1)),v,'green');
            hold on
            
            subplot(3,1,3);
            plot(t+sum(tau(1:i-1)),a,'green');
            hold on
        end
        
        subplot(3,1,1);
        xlabel('t','FontSize',16)
        ylabel('s','FontSize',16)
        subplot(3,1,2);
        xlabel('t','FontSize',16)
        ylabel('v','FontSize',16)
        subplot(3,1,3);
        xlabel('t','FontSize',16)
        ylabel('a','FontSize',16)
        
    case 'Bezier_Curve'
        
        t = linspace(0,1,100);
        [Q,Qs,Qss] = Bezier( p,t );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(p,2)-1
            plot3(Q(1,:),Q(2,:),Q(3,:),'blue','LineWidth',2)
        end
        
        % Position, Geschwindigkeit, Beschleunigung am Beispiel der x-Richtung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        for i = 1 : size(p,2)-1
            j = 1;
            s = Q(j,:);
            v = Qs(j,:);
            a = Qss(j,:);
            
            subplot(3,1,1);
            plot(t*T_ges,s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(t*T_ges,v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(t*T_ges,a,'blue');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 2;
            s = Q(j,:);
            v = Qs(j,:);
            a = Qss(j,:);
            
            subplot(3,1,1);
            plot(t*T_ges,s,'red');
            hold on
            
            subplot(3,1,2);
            plot(t*T_ges,v,'red');
            hold on
            
            subplot(3,1,3);
            plot(t*T_ges,a,'red');
            hold on
        end
        
        for i = 1 : size(p,2)-1
            j = 3;
            s = Q(j,:);
            v = Qs(j,:);
            a = Qss(j,:);
            
            subplot(3,1,1);
            plot(t*T_ges,s,'green');
            hold on
            
            subplot(3,1,2);
            plot(t*T_ges,v,'green');
            hold on
            
            subplot(3,1,3);
            plot(t*T_ges,a,'green');
            hold on
        end

    case 'Linear_Interpolation_with_Polynomial_Blends'
        
        [ a0,a1,a2,a3,a4,a5,tau,pn ] = Linear_Interpolation_with_Polynomial_Blends( p,T_ges,r );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(pn(1,:),pn(2,:),pn(3,:),'MarkerEdgeColor','black')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(pn,2)-1
            t = linspace(0,r((i-rem(i,2))/2+1),100);
            px = [a5(1,i),a4(1,i),a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a5(2,i),a4(2,i),a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a5(3,i),a4(3,i),a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
        end

    case 'Linear_Interpolation_with_Polynomial4_Blends'
        
        [ a0,a1,a2,a3,a4,p0,p1,p2,p3,p4,t0,t4,q,q1,q2 ] = Linear_Interpolation_with_Polynomial4_Blends( p,r );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        hold on
        scatter3(p0(1,:),p0(2,:),p0(3,:),15,'MarkerEdgeColor','green')
        scatter3(p1(1,:),p1(2,:),p1(3,:),15,'MarkerEdgeColor','green')
        scatter3(p3(1,:),p3(2,:),p3(3,:),15,'MarkerEdgeColor','green')
        scatter3(p4(1,:),p4(2,:),p4(3,:),15,'MarkerEdgeColor','green')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : 2*size(p,2)-3
            t = linspace(0,1,100);
            px = [a4(1,i),a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a4(2,i),a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a4(3,i),a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
            for k = 1 : 99
                s(k) = norm([x(k) y(k) z(k)]-[x(k+1) y(k+1) z(k+1)]);
            end
            arclength(i) = sum(s);
        end
        
        tau = zeros(1,2*size(p,2)-3);
        for i = 1 : 2*size(p,2)-3
            tau(i) = arclength(i)/sum(sum(arclength))*T_ges;
        end
        
        % Position, Geschwindigkeit, Beschleunigung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        % Umparametrisierung
        for i = 0 : 2*size(p,2)-4
            if rem(i,2) == 0
                p0_dach(:,i+2) = p0(:,i/2+2);
                p1_dach(:,i+1) = p0(:,i/2+2);
            else
                p0_dach(:,i+2) = p4(:,(i+1)/2+1);
                p1_dach(:,i+1) = p1(:,(i+1)/2+1);
            end
        end
        p0_dach(:,1) = p0(:,1);
        for i = 0 : 2*size(p,2)-4
            if rem(i,2) == 0
                lambda(i+1) = norm(p1_dach(:,i+1) - p0_dach(:,i+1));
            else
                lambda(i+1) = 4*norm(p1_dach(:,i+1) - p0_dach(:,i+1));
            end
        end
        for i = 0 : 2*size(p,2)-3
            if i == 0
                u_dach(i+1) = 0;
            else
                u_dach(i+1) =u_dach(i) + lambda(i);
            end
        end
        
        % Skalierungsfaktor
        sk = T_ges/u_dach(2*size(p,2)-2);
        
        for i = 1 : 2*size(p,2)-3
            j = 1;
            t = linspace(0,1,100);
            px = [a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'blue');
            hold on
        end
        
        for i = 1 : 2*size(p,2)-3
            j = 2;
            t = linspace(0,1,100);
            px = [a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'red');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'red');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'red');
            hold on
        end
        
        for i = 1 : 2*size(p,2)-3
            j = 3;
            t = linspace(0,1,100);
            px = [a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'green');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'green');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'green');
            hold on
        end

    case 'Linear_Interpolation_with_Polynomial5_Blends'
        
        [ a0,a1,a2,a3,a4,a5,p0,p1,p2,p3,p4,p5,t0,t5,q,q1,q2 ] = Linear_Interpolation_with_Polynomial5_Blends( p,r );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        hold on
        scatter3(p0(1,:),p0(2,:),p0(3,:),15,'MarkerEdgeColor','green')
        scatter3(p1(1,:),p1(2,:),p1(3,:),15,'MarkerEdgeColor','green')
        scatter3(p2(1,:),p2(2,:),p2(3,:),15,'MarkerEdgeColor','green')
        scatter3(p3(1,:),p3(2,:),p3(3,:),15,'MarkerEdgeColor','green')
        scatter3(p4(1,:),p4(2,:),p4(3,:),15,'MarkerEdgeColor','green')
        scatter3(p5(1,:),p5(2,:),p5(3,:),15,'MarkerEdgeColor','green')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        hold on
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : 2*size(p,2)-3
            t = linspace(0,1,100);
            px = [a5(1,i),a4(1,i),a3(1,i),a2(1,i),a1(1,i),a0(1,i)];
            py = [a5(2,i),a4(2,i),a3(2,i),a2(2,i),a1(2,i),a0(2,i)];
            pz = [a5(3,i),a4(3,i),a3(3,i),a2(3,i),a1(3,i),a0(3,i)];
            x = polyval(px,t);
            y = polyval(py,t);
            z = polyval(pz,t);
            plot3(x,y,z,'blue','LineWidth',2)
        end
        
        % Position, Geschwindigkeit, Beschleunigung
        figure2=figure(2);
        set(gca,'Box','off')
        set(figure2,'Units','normalized','OuterPosition',[0.5 0.05 0.5 0.95],...
            'Name','Verlauf','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        
        % Umparametrisierung
        for i = 0 : 2*size(p,2)-4
            if rem(i,2) == 0
                p0_dach(:,i+2) = p0(:,i/2+2);
                p1_dach(:,i+1) = p0(:,i/2+2);
            else
                p0_dach(:,i+2) = p5(:,(i+1)/2+1);
                p1_dach(:,i+1) = p1(:,(i+1)/2+1);
            end
        end
        p0_dach(:,1) = p0(:,1);
        for i = 0 : 2*size(p,2)-4
            if rem(i,2) == 0
                lambda(i+1) = norm(p1_dach(:,i+1) - p0_dach(:,i+1));
            else
                lambda(i+1) = 5*norm(p1_dach(:,i+1) - p0_dach(:,i+1));
            end
        end
        for i = 0 : 2*size(p,2)-3
            if i == 0
                u_dach(i+1) = 0;
            else
                u_dach(i+1) =u_dach(i) + lambda(i);
            end
        end
        
        % Skalierungsfaktor
        sk = T_ges/u_dach(2*size(p,2)-2);
        
        for i = 1 : 2*size(p,2)-3
            j = 1;
            t = linspace(0,1,100);
            px = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'blue');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'blue');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'blue');
            hold on
        end
        
        for i = 1 : 2*size(p,2)-3
            j = 2;
            t = linspace(0,1,100);
            px = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'red');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'red');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'red');
            hold on
        end
        
        for i = 1 : 2*size(p,2)-3
            j = 3;
            t = linspace(0,1,100);
            px = [a5(j,i),a4(j,i),a3(j,i),a2(j,i),a1(j,i),a0(j,i)];
            pxd = polyder(px);
            pxdd = polyder(pxd);
            s = polyval(px,t);
            v = polyval(pxd,t)/lambda(i);
            a = polyval(pxdd,t)/lambda(i);
            
            subplot(3,1,1);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,s,'green');
            hold on
            
            subplot(3,1,2);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,v,'green');
            hold on
            
            subplot(3,1,3);
            plot(linspace(0,u_dach(i+1)-u_dach(i),100)*sk+u_dach(i)*sk,a,'green');
            hold on
        end
        
    case 'Linear_Interpolation_with_Bezier_Blends'
        
        [ p0,p1,p2,p3,p4,t0,t4,q,q1,q2 ] = Linear_Interpolation_with_Bezier_Blends( p,r );
        
        % grafische Darstellung
        figure1=figure(1);
        set(gca,'Box','off')
        set(figure1,'Units','normalized','OuterPosition',[0.0 0.05 0.5 0.95],...
            'Name','Trajektorie','NumberTitle','off','Toolbar','figure','PaperPositionMode','auto');
        scatter3(p(1,:),p(2,:),p(3,:),'MarkerEdgeColor','black')
        hold on
        scatter3(p0(1,:),p0(2,:),p0(3,:),15,'MarkerEdgeColor','green')
        scatter3(p1(1,:),p1(2,:),p1(3,:),15,'MarkerEdgeColor','green')
        scatter3(p3(1,:),p3(2,:),p3(3,:),15,'MarkerEdgeColor','green')
        scatter3(p4(1,:),p4(2,:),p4(3,:),15,'MarkerEdgeColor','green')
        title ('Trajektorie','FontSize',16)
        xlabel('x','FontSize',16)
        ylabel('y','FontSize',16)
        zlabel('z','FontSize',16)
        axis equal
        plot3(p(1,:),p(2,:),p(3,:),'red')
        
        for i = 1 : size(p,2)-1
            t = linspace(0,1,100);
            for j = 1 : length(t)
                l(:,j) = q2(:,i) + (q1(:,i+1) - q2(:,i)) * t(j);
            end
            plot3(l(1,:),l(2,:),l(3,:),'blue','LineWidth',2)
            p_blends = [p0(:,i),p1(:,i),p2(:,i),p3(:,i),p4(:,i)];
            Q = Bezier(p_blends,t);
            plot3(Q(1,:),Q(2,:),Q(3,:),'blue','LineWidth',2)
        end
        
end

Berechnungsdauer = toc