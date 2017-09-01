    addpath(genpath(pwd)); % Enalbe folder "GUI_Reconbot"
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q0 = 0; 
    q0q1q2_0 = [0, 0, pi/4, pi/2, -pi/4, 0, 0, pi/4, pi/2, -pi/4, 0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_0 = [0 0 208.879343162506 0 0 0, 0 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% ---------------------
    IK_ModeSelection_GUI;
    %%--------------------------
    
    %%
    if isreal(q1q2) == 0 || (WSvalue(1) == 0 && WSvalue(2) == 0 && WSvalue(3) == 0)
        
        errordlg('Inputs exceed workspace, Please Modify!','Workspace Error');
%         pause(2);
%         close;
%         for i=1:8
%            EditList{i}=['edit',num2str(i)];
%            TextList{i}=['text',num2str(i)];
%         end
% 
%         for i=1:8
%            set(handles.(EditList{i}),'enable','on');
%            set(handles.(EditList{i}),'string','');
%            set(handles.(TextList{i}),'enable','on');
%         end
%         
%         set(handles.text1,'string','X');
%         set(handles.text2,'string','Y');
%         set(handles.text3,'string','Z');
%         set(handles.text4,'string','?');
%         set(handles.text5,'string','?11');
%         set(handles.text6,'string','?12');
%         set(handles.text7,'string','?21');
%         set(handles.text8,'string','?22');

    else
    end
    