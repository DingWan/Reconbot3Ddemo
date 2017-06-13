figure(3)
Origin = [ 0 0 0 ];
Direction1 = S(1,1) * 1 * [Origin; U(1:3,1)'];
plot3(Direction1(:,1), Direction1(:,2), Direction1(:,3), 'r-'); hold on

Direction2 = S(2,2) * 1 * [Origin; U(1:3,2)'];
plot3(Direction2(:,1), Direction2(:,2), Direction2(:,3), 'g-'); hold on

Direction3 = S(3,3) * 1 * [Origin; U(1:3,3)'];
plot3(Direction3(:,1), Direction3(:,2), Direction3(:,3), 'b-');hold on

Direction4 = S(4,4) * 1000 * [Origin; U(1:3,4)'];
plot3(Direction4(:,1), Direction4(:,2), Direction4(:,3), 'k-');hold on

grid on
axis equal


%% --------------------Plot the mechanism Ai Bi Ci------------------
            PA1B1C1x = [A1(1), B1(1), C1(1)];
            PA1B1C1y = [A1(2), B1(2), C1(2)];
            PA1B1C1z = [A1(3), B1(3), C1(3)];
            plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
            
            PA2B2C2x = [A2(1), B2(1), C2(1)];
            PA2B2C2y = [A2(2), B2(2), C2(2)];
            PA2B2C2z = [A2(3), B2(3), C2(3)];
            plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
            
            PC1C2x = [C1(1), C2(1)];
            PC1C2y = [C1(2), C2(2)];
            PC1C2z = [C1(3), C2(3)];
            plot3(PC1C2x, PC1C2y, PC1C2z,'g-','linewidth',3); hold on;
            
            PA1A2x = [A1(1), A2(1)];
            PA1A2y = [A1(2), A2(2)];
            PA1A2z = [A1(3), A2(3)];
            plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
            
            grid on;
            axis equal;