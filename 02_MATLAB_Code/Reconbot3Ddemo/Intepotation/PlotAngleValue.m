figure(2)

Time = 4;%Second
increment = Time/length(q0q1q2_mat);
subplot(2,1,1);
i = 0:increment:Time-increment;
q0q1q2_mat_Angle = q0q1q2_mat * 180/pi;
plot(i,q0q1q2_mat_Angle(:,2),'r'); hold on
plot(i,q0q1q2_mat_Angle(:,3),'g'); hold on
plot(i,q0q1q2_mat_Angle(:,5),'b'); hold on
plot(i,q0q1q2_mat_Angle(:,4),'m'); hold on
plot(i,q0q1q2_mat_Angle(:,6),'k'); hold on
title('Subplot 1: Branch Chain A1C1, q11(Red), q12(Green), q14(Blue)')

subplot(2,1,2);
plot(i,q0q1q2_mat_Angle(:,7),'r'); hold on
plot(i,q0q1q2_mat_Angle(:,8),'g'); hold on
plot(i,q0q1q2_mat_Angle(:,9),'b'); hold on
plot(i,q0q1q2_mat_Angle(:,10),'m'); hold on
plot(i,q0q1q2_mat_Angle(:,11),'k'); hold on
title('Subplot 1: Branch Chain A2C2, q21(Red), q22(Green), q23(Blue)')