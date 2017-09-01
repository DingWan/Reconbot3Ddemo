

function RCB_ABC_plot3(p, EulerAngle_q11_theta, ABC, q0)

% Link connection point displacement
        BaseLow_CenterPointBearing = [250, 250, 88.97];
        %
        BaseUP_CenterPointBearing = [170, 160, 5.47];
        BaseUP_CenterPointA1C1 = [170, 50, 58.88];
        %
        BaseJointA1C1A2C2_CenterPointMotor = [24, 48.10, 4.5];
        BaseJointA1C1A2C2_CenterPointSidePlate = [0, 48.10, 28];

        Displacement = [BaseLow_CenterPointBearing(1:2),...
            BaseLow_CenterPointBearing(3) + BaseUP_CenterPointA1C1(3) - BaseUP_CenterPointBearing(3)...
            + BaseJointA1C1A2C2_CenterPointSidePlate(3) - BaseJointA1C1A2C2_CenterPointMotor(3)];
        p = p(1:3) + Displacement;
        for i = 1:length(ABC(:,1))
            A1(i,:) = ABC(i,1:3);
            B1(i,:) = ABC(i,4:6);
            C1(i,:) = ABC(i,7:9);
            A2(i,:) = ABC(i,10:12);
            B2(i,:) = ABC(i,13:15);
            C2(i,:) = ABC(i,16:18);  
            A1(i,:) = (rotz(q0) * A1(i,:)' + Displacement')';
            B1(i,:) = (rotz(q0) * B1(i,:)' + Displacement')';
            C1(i,:) = (rotz(q0) * C1(i,:)' + Displacement')';
            A2(i,:) = (rotz(q0) * A2(i,:)' + Displacement')';
            B2(i,:) = (rotz(q0) * B2(i,:)' + Displacement')';
            C2(i,:) = (rotz(q0) * C2(i,:)' + Displacement')';
        end

for i = 1:length(ABC(:,1))
    %% --------------------Plot the mechanism Ai Bi Ci------------------
    PA1B1C1x = [A1(i,1), B1(i,1), C1(i,1)];
    PA1B1C1y = [A1(i,2), B1(i,2), C1(i,2)];
    PA1B1C1z = [A1(i,3), B1(i,3), C1(i,3)];
    plot3(PA1B1C1x, PA1B1C1y, PA1B1C1z,'b-'); hold on;
    
    PA2B2C2x = [A2(i,1), B2(i,1), C2(i,1)];
    PA2B2C2y = [A2(i,2), B2(i,2), C2(i,2)];
    PA2B2C2z = [A2(i,3), B2(i,3), C2(i,3)];
    plot3(PA2B2C2x, PA2B2C2y, PA2B2C2z,'r-'); hold on;
    
    PC1C2x = [C1(i,1), C2(i,1)];
    PC1C2y = [C1(i,2), C2(i,2)];
    PC1C2z = [C1(i,3), C2(i,3)];
    plot3(PC1C2x, PC1C2y, PC1C2z,'k-','linewidth',3); hold on;
    
    PA1A2x = [A1(i,1), A2(i,1)];
    PA1A2y = [A1(i,2), A2(i,2)];
    PA1A2z = [A1(i,3), A2(i,3)];
    plot3(PA1A2x, PA1A2y, PA1A2z,'k-','linewidth',3); hold on;
    
    %----------------- plot xyz axes of base point --------------
    x_axis = [50 0 0];
    y_axis = [0 50 0];
    z_axis = [0 0 50];
    OP= [0 0 0];    
    xyz = [OP;x_axis;OP;y_axis;OP;z_axis];
    xyz_disp = xyz + [Displacement;Displacement;Displacement;Displacement;Displacement;Displacement];
    jj = 1:2;
    plot3(xyz_disp(jj,1),xyz_disp(jj,2),xyz_disp(jj,3),'-r','LineWidth',2); hold on
    jj = 3:4;
    plot3(xyz_disp(jj,1),xyz_disp(jj,2),xyz_disp(jj,3),'-g','LineWidth',2); hold on
    jj = 5:6;
    plot3(xyz_disp(jj,1),xyz_disp(jj,2),xyz_disp(jj,3),'-b','LineWidth',2); hold on
    %-----------------------------------------------------------
    %------------------plot xyz axes of Moving Platform----------------
    %RotationMatrix_from_axis_angle1 = eul2rotm(EulerAngle,'ZYX');
    RotationMatrix_from_axis_angle1 = eul2rotm(EulerAngle_q11_theta(1:3),'ZYX');
    xyz = [p(1:3);p(1:3);p(1:3);p(1:3);p(1:3);p(1:3)] + transpose(RotationMatrix_from_axis_angle1 * transpose(xyz));
    jj = 1:2;
    plot3(xyz(jj,1),xyz(jj,2),xyz(jj,3),'-r','LineWidth',2);
    jj = 3:4;
    plot3(xyz(jj,1),xyz(jj,2),xyz(jj,3),'-g','LineWidth',2);
    jj = 5:6;
    plot3(xyz(jj,1),xyz(jj,2),xyz(jj,3),'-b','LineWidth',2);
    hold on;
    axis equal;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    %----------------------------------------------
    %hold off
    %view(90,90);
end
end