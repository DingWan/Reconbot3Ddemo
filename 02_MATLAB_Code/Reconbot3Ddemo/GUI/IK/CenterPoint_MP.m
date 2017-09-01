 %================== Center Point of Moving Platform ===================
    %ReconbotANI(q0q1q2_mat(n*(IntepPointNum-1)+i,:));
    Displacement = [250,250,167.4400];
    p_MP_0 = HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Pos_mat(:,1:3);
    [m,n]=size(HomePos2SelectedEndPos_OutputData_RePlan.CartesianSpace.MP_Pos_mat);
    p_MP = p_MP_0 + Displacement(ones(m,1),:);
    x=p_MP(:,1);
    y=p_MP(:,2);
    z=p_MP(:,3);
%     plot3(x,y,z);
    handles = getappdata(0,'patch_h');
    Tr = handles(12);
%     trail = 'y';
%     if trail == 'y'
%         x_trail = getappdata(0,'xtrail');
%         y_trail = getappdata(0,'ytrail');
%         z_trail = getappdata(0,'ztrail');
%         %
%         xTrdata = [x_trail p_MP(1)];
%         yTrdata = [y_trail p_MP(2)];
%         zTrdata = [z_trail p_MP(3)];
        %
%         setappdata(0,'xtrail',xTrdata); % used for trail tracking.
%         setappdata(0,'ytrail',yTrdata); % used for trail tracking.
%         setappdata(0,'ztrail',zTrdata); % used for trail tracking.
        %
        set(Tr,'xdata',x,'ydata',y,'zdata',z);
    