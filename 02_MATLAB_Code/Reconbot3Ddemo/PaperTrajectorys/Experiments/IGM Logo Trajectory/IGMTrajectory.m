   
   %IGM = [P(:,1)-80*ones(length(P),1), 50 * ones(length(P),1),P(:,2)+150*ones(length(P),1)];
   
   IGM = [ -80    50   150
           -60    50   150
           -60    50   210
           -80    50   210
           -40    50   210
           -60    50   210
           -60    50   150
           -20    50   150
           -20    50   210
            20    50   210
           -20    50   210
           -20    50   150
            20    50   150
            20    50   170
            20    50   150
            40    50   150
            50    50   210
            60    50   182
            70    50   210
            80    50   150
           -80    50   150
         ];  
   %----------------- plot IGM offset --------------
    Displacement = [250,250,83.5+60.44+(45.5-22)];
    for i = 1:1:21
      IGM_offset(i,:) = (rotz(90) * IGM(i,:)')' + Displacement;
    end
    
   %% 
   %figure(2)
   i = 1:1:21;
   %plot3(IGM(i,1),IGM(i,2),IGM(i,3)); hold on;
   plot3(IGM_offset(i,1),IGM_offset(i,2),IGM_offset(i,3)); hold on;
   axis equal
   
   