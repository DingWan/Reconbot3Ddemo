% clear all
% clf

q0q1q2 = [0, 0, pi/3, pi/3, pi/6, 0, 0, pi/3, pi/3, pi/6, 0] * 180/pi;
[LinkA1C1_ColChe, LinkA2C2_ColChe, BaseUpPlatform_ColChe] = CollisionPointsFK(q0q1q2);

% LinkA1C1_ColChe with BaseUpPlatform_ColChe
tic
for i = 1 : length(LinkA1C1_ColChe(:,1))
    for j = 1 : length(BaseUpPlatform_ColChe(:,1))

        P1 = LinkA1C1_ColChe(i,1:3);           P2 = LinkA1C1_ColChe(i,4:6);
        P3 = BaseUpPlatform_ColChe(j,1:3);     P4 = BaseUpPlatform_ColChe(j,4:6);        
        
        [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
        LinkA1C1_BaseUpPlatform_distance(i,j) = distance;

    end
end

% LinkA2C2_ColChe with BaseUpPlatform_ColChe
for i = 1 : length(LinkA2C2_ColChe(:,1))
    for j = 1 : length(BaseUpPlatform_ColChe(:,1))

        P1 = LinkA2C2_ColChe(i,1:3);           P2 = LinkA2C2_ColChe(i,4:6);
        P3 = BaseUpPlatform_ColChe(j,1:3);     P4 = BaseUpPlatform_ColChe(j,4:6);        
        
        [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
        LinkA2C2_BaseUpPlatform_distance(i,j) = distance;

    end
end

% LinkA1C1_ColChe with LinkA2C2_ColChe
for i = 1 : length(LinkA1C1_ColChe(:,1))
    for j = 1 : length(LinkA2C2_ColChe(:,1))

        P1 = LinkA1C1_ColChe(i,1:3);     P2 = LinkA1C1_ColChe(i,4:6);
        P3 = LinkA2C2_ColChe(j,1:3);     P4 = LinkA2C2_ColChe(j,4:6);        
        
        [distance, ~] = DistBetween2Segment(P1, P2, P3, P4);
        LinkA1C1_LinkA2C2_ColChe_distance(i,j) = distance;

    end
end
toc

Total_distance = [LinkA1C1_BaseUpPlatform_distance, LinkA2C2_BaseUpPlatform_distance, LinkA1C1_LinkA2C2_ColChe_distance];

[rows,cols] = find(Total_distance == min(min(Total_distance)));

if cols(1) < 13
    P1 = LinkA1C1_ColChe(rows(1),1:3);           P2 = LinkA1C1_ColChe(rows(1),4:6);
    P3 = BaseUpPlatform_ColChe(cols(1),1:3);     P4 = BaseUpPlatform_ColChe(cols(1),4:6);
elseif cols(1) > 12 && cols(1) < 25
    P1 = LinkA2C2_ColChe(rows(1),1:3);           P2 = LinkA2C2_ColChe(rows(1),4:6);
    P3 = BaseUpPlatform_ColChe(cols(1),1:3);     P4 = BaseUpPlatform_ColChe(cols(1),4:6);    
elseif cols(1) > 24    
    P1 = LinkA1C1_ColChe(rows(1),1:3);           P2 = LinkA1C1_ColChe(rows(1),4:6);
    P3 = LinkA2C2_ColChe(cols(1),1:3);           P4 = LinkA2C2_ColChe(cols(1),4:6);   
end

[distance, ClosePoint] = DistBetween2Segment(P1, P2, P3, P4);
P12 = [P1;P2];
P34 = [P3;P4];
CP = [ClosePoint{2};ClosePoint{3}];
plot3(P12(:,1),P12(:,2),P12(:,3),'r-');hold on
plot3(P34(:,1),P34(:,2),P34(:,3),'r-');hold on
plot3(CP(:,1),CP(:,2),CP(:,3),'r-o');hold on
axis equal