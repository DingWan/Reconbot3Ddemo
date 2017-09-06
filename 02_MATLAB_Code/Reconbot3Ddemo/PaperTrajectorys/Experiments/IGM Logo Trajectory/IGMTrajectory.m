   
   %IGM = [P(:,1)-80*ones(length(P),1), 50 * ones(length(P),1),P(:,2)+150*ones(length(P),1)];
   
   IGM = [ -80    50    80
           -60    50    80
           -60    50   140
           -80    50   140
           -40    50   140
           -60    50   140
           -60    50    80
           -20    50    80
           -20    50   140
            20    50   140
           -20    50   140
           -20    50    80
            20    50    80
            20    50   100
            20    50    80
            40    50    80
            50    50   140
            60    50   112
            70    50   140
            80    50    80
           -80    50    80
         ];  
       
     IGM = 1.2 * IGM;
     IGM(:,3) = IGM(:,3)+60*ones(21,1);
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
   
   