 

%========================== Collision Check ============================
    [CollisionOccur_DistanceA1C1(NumIntepoPoints*(i-1)+j,:), CollisionPoints_A1C1, CollisionOccur_DistanceA2C2(NumIntepoPoints*(i-1)+j,:), CollisionPoints_A2C2]...
        = CollisionCheck(q0q1q2_Optimal_SingleRow * 180/pi);    
    %============================= End =================================
    
     %========================== Collision Line ============================
     % In file "InitHome.m", we use "setappdata(obj,name,val)" to set obj = 0; 
     % L1 = patch('faces', BaseLow_data.F1, 'vertices' ,Link_BaseLow(:,1:3));
     % Li = ... (i = 1-11, represent all links)
     % Tr = plot3(0,0,0,'b.'); % holder for trail paths
     % CPsA1C1 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A1C1
     % CPsA2C2 = plot3([0 0],[0 0],[0 0],'r-'); % Collision Points for chain A2C2
     % setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,L8,L9,L10,L11,Tr,CPsA1C1,CPsA2C2])
     
     % setappdata(0,'xCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
     % setappdata(0,'yCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.
     % setappdata(0,'zCollisionPoints_A1C1',[0;0]); % used for chain A1C1 Collision detecting.

     % setappdata(0,'xCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
     % setappdata(0,'yCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
     % setappdata(0,'zCollisionPoints_A2C2',[0;0]); % used for chain A2C2 Collision detecting.
 
    handles = getappdata(0,'patch_h');
    CPsA1C1 = handles(13);
    CPsA2C2 = handles(14);
    CollisionDetection = 'y';
    if CollisionDetection == 'y'
        x_CollisionPoints_A1C1 = getappdata(0,'xCollisionPoints_A1C1');
        y_CollisionPoints_A1C1 = getappdata(0,'yCollisionPoints_A1C1');
        z_CollisionPoints_A1C1 = getappdata(0,'zCollisionPoints_A1C1');
        %
        x_CollisionPoints_A2C2 = getappdata(0,'xCollisionPoints_A2C2');
        y_CollisionPoints_A2C2 = getappdata(0,'yCollisionPoints_A2C2');
        z_CollisionPoints_A2C2 = getappdata(0,'zCollisionPoints_A2C2');
        %
        xCPsA1C1data = [x_CollisionPoints_A1C1 CollisionPoints_A1C1(:,1)];
        yCPsA1C1data = [y_CollisionPoints_A1C1 CollisionPoints_A1C1(:,2)];
        zCPsA1C1data = [z_CollisionPoints_A1C1 CollisionPoints_A1C1(:,3)];
        %
        xCPsA2C2data = [x_CollisionPoints_A2C2 CollisionPoints_A2C2(:,1)];
        yCPsA2C2data = [y_CollisionPoints_A2C2 CollisionPoints_A2C2(:,2)];
        zCPsA2C2data = [z_CollisionPoints_A2C2 CollisionPoints_A2C2(:,3)];
        %
        setappdata(0,'xCollisionPoints_A1C1',xCPsA1C1data); % used for trail tracking.
        setappdata(0,'yCollisionPoints_A1C1',yCPsA1C1data); % used for trail tracking.
        setappdata(0,'zCollisionPoints_A1C1',zCPsA1C1data); % used for trail tracking.
        %
        setappdata(0,'xCollisionPoints_A2C2',xCPsA2C2data); % used for trail tracking.
        setappdata(0,'yCollisionPoints_A2C2',yCPsA2C2data); % used for trail tracking.
        setappdata(0,'zCollisionPoints_A2C2',zCPsA2C2data); % used for trail tracking.
        %
        set(CPsA1C1,'xdata',xCPsA1C1data(:,i+1),'ydata',yCPsA1C1data(:,i+1),'zdata',zCPsA1C1data(:,i+1),'Color','red', 'LineStyle','-'); %hold off
        set(CPsA2C2,'xdata',xCPsA2C2data(:,i+1),'ydata',yCPsA2C2data(:,i+1),'zdata',zCPsA2C2data(:,i+1),'Color','red', 'LineStyle','-'); %hold off
    end
    %============================= End =================================
    
    %================== Center Point of Moving Platform ===================
%     ReconbotANI(q0q1q2_mat(n*(IntepPointNum-1)+i,:));
    Displacement = [250,250,167.4400];
    p_MP = p(1:3) + Displacement;
    handles = getappdata(0,'patch_h');
    Tr = handles(12);
    trail = 'y';
    if trail == 'y'
        x_trail = getappdata(0,'xtrail');
        y_trail = getappdata(0,'ytrail');
        z_trail = getappdata(0,'ztrail');
        %
        xTrdata = [x_trail p_MP(1)];
        yTrdata = [y_trail p_MP(2)];
        zTrdata = [z_trail p_MP(3)];
        %
        setappdata(0,'xtrail',xTrdata); % used for trail tracking.
        setappdata(0,'ytrail',yTrdata); % used for trail tracking.
        setappdata(0,'ztrail',zTrdata); % used for trail tracking.
        %
        set(Tr,'xdata',xTrdata,'ydata',yTrdata,'zdata',zTrdata);
    end