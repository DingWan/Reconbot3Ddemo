function clr_trail_CollisionPoints_button_press
        %disp('pushed clear trail bottom');
        handles = getappdata(0,'patch_h');           %
        Tr = handles(12);
        CPsA1C1 = handles(13);
        CPsA2C2= handles(14);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        setappdata(0,'xCollisionPoints_A1C1',[0;0]); % used for Collision detecting.
        setappdata(0,'yCollisionPoints_A1C1',[0;0]); % used for Collision detecting.
        setappdata(0,'zCollisionPoints_A1C1',[0;0]); % used for Collision detecting.
        %
        setappdata(0,'xCollisionPoints_A2C2',[0;0]); % used for Collision detecting.
        setappdata(0,'yCollisionPoints_A2C2',[0;0]); % used for Collision detecting.
        setappdata(0,'zCollisionPoints_A2C2',[0;0]); % used for Collision detecting.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        set(CPsA1C1,'xdata',[0;0],'ydata',[0;0],'zdata',[0;0]);
        set(CPsA2C2,'xdata',[0;0],'ydata',[0;0],'zdata',[0;0]);
    end