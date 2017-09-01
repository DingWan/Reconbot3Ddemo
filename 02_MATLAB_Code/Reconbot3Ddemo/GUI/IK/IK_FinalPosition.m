Global_Para;

IK_ModeSelection_GUI;

SolutionRow=str2num(get(handles.edit_RCB,'string'));

if SolutionRow >= 1 && SolutionRow <= length(q1q2(:,1))
    
   q0q1q2_display = [0, q1q2(SolutionRow,:)];
   q0q1q2_all = [zeros(length(q1q2(:,1)),1), q1q2];
   ReconbotANI(q0q1q2_display);
 
else

    errordlg('index exceeds matrix dimensions, Please Modify!','Input Error');
    
end