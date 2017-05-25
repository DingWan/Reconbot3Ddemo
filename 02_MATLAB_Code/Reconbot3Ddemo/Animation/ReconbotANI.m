function ReconbotANI(q0q1q2)
        % This function will animate the Puma 762 robot given joint angles.
        % n is number of steps for the animation
        % trail is 'y' or 'n' (n = anything else) for leaving a trail.
        %
        %disp('System in animate...');

        n = length(q0q1q2(:,1));
        for i = 1:1:n
            % Forward Kinematics
            [T_01,T_1_02,T_1_03,T_1_04,T_1_05,T_1_06,T_2_02,T_2_03,T_2_04,T_2_05] = RCB_3Ddemo_ForwardKinematics(q0q1q2(i,:)*180/pi);
            %-------------------------------------------------------
            %q0q1q2(i,:)*180/pi;
%             BaseLow_data = getappdata(0,'Link_BaseLow_data');
%             BaseUP_data = getappdata(0,'Link_BaseUP_data');
            BaseJointA1C1_data = getappdata(0,'Link_BaseJointA1C1_data');
            LowLinkA1C1_data = getappdata(0,'Link_LowLinkA1C1_data');
            UpLinkA1C1_data = getappdata(0,'Link_UpLinkA1C1_data');
            UPjointA1C1_data = getappdata(0,'Link_UPjointA1C1_data');
            BaseJointA2C2_data = getappdata(0,'Link_BaseJointA2C2_data');
            LowLinkA2C2_data = getappdata(0,'Link_LowLinkA2C2_data');
            UpLinkA2C2_data = getappdata(0,'Link_UpLinkA2C2_data');
            UPjointA2C2_data = getappdata(0,'Link_UPjointA2C2_data');
            MovingPlatform_data = getappdata(0,'Link_MovingPlatform_data');        
            % 
            
            % Actual vertex data of robot links
%             Link_BaseLow        =           BaseLow_data.V1;
%             Link_BaseUP         = (T_01   * BaseUP_data.V2')';
            Link_BaseJointA1C1  = (T_1_02 * BaseJointA1C1_data.V3')';
            Link_LowLinkA1C1    = (T_1_03 * LowLinkA1C1_data.V4')';
            Link_UpLinkA1C1     = (T_1_04 * UpLinkA1C1_data.V5')';
            Link_UPjointA1C1    = (T_1_05 * UPjointA1C1_data.V6')';

            Link_MovingPlatform = (T_1_06 * MovingPlatform_data.V11')';

            Link_BaseJointA2C2  = (T_2_02 * BaseJointA2C2_data.V7')';
            Link_LowLinkA2C2    = (T_2_03 * LowLinkA2C2_data.V8')';
            Link_UpLinkA2C2     = (T_2_04 * UpLinkA2C2_data.V9')';
            Link_UPjointA2C2    = (T_2_05 * UPjointA2C2_data.V10')';
            %
            handles = getappdata(0,'patch_h');           %
            L1 = handles(1);
            L2 = handles(2);
            L3 = handles(3);
            L4 = handles(4);
            L5 = handles(5);
            L6 = handles(6);
            L7 = handles(7);
            L8 = handles(8);
            L9 = handles(9);
            L10 = handles(10);
            L11 = handles(11);
            %
            setappdata(0,'xtrail',0); % used for trail tracking.
            setappdata(0,'ytrail',0); % used for trail tracking.
            setappdata(0,'ztrail',0); % used for trail tracking.
            %            
%             set(L1,'vertices',Link_BaseLow(:,1:3), 'facec', [105 105 105]/255);%105 105 105
%             set(L1, 'EdgeColor','none');

%             set(L2, 'vertices' ,Link_BaseUP(:,1:3), 'facec', [250 235 215]/255);
%             set(L2, 'EdgeColor','none');
            set(L3, 'vertices' ,Link_BaseJointA1C1(:,1:3), 'facec', [105 105 105]/255);
            set(L3, 'EdgeColor','none');
            set(L4, 'vertices' ,Link_LowLinkA1C1(:,1:3), 'facec', [255 182 193]/255);
            set(L4, 'EdgeColor','none');
            set(L5, 'vertices' ,Link_UpLinkA1C1(:,1:3), 'facec', [135 206 250]/255);
            set(L5, 'EdgeColor','none');
            set(L6, 'vertices' ,Link_UPjointA1C1(:,1:3), 'facec', [105 105 105]/255);
            set(L6, 'EdgeColor','none');

            set(L11, 'vertices' ,Link_MovingPlatform(:,1:3), 'facec', [250 235 215]/255);
            set(L11, 'EdgeColor','none');

            set(L7, 'vertices' ,Link_BaseJointA2C2(:,1:3), 'facec', [105 105 105]/255);
            set(L7, 'EdgeColor','none');
            set(L8, 'vertices' ,Link_LowLinkA2C2(:,1:3), 'facec', [255 182 193]/255);
            set(L8, 'EdgeColor','none');
            set(L9, 'vertices' ,Link_UpLinkA2C2(:,1:3), 'facec', [135 206 250]/255);
            set(L9, 'EdgeColor','none');
            set(L10, 'vertices' ,Link_UPjointA2C2(:,1:3), 'facec', [105 105 105]/255);
            set(L10, 'EdgeColor','none');
            %
            
            %-------------------------------------------------------
            drawnow %update 
        end
        
    end