function [ Pos_Intep, Vel_Intep, Acc_Intep, time_Intep ] =  IntepotationP2P_debugging(Mode, PosOri_previous,q0q1q2_previous_trajpoint, PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, Time,l1, l2)

%% =================== Catisian Space Trajctory Planning ====================
PosOri = PosOri_current;
for i = 1:6
    if isempty(PosOri_previous{i}) == 1
        PosOri_previous{i} = 0;
    end
    if isempty(PosOri_current{i}) == 1
        PosOri_current{i} = 0;
    end
end
n = NumIntepoPoints;
for i = 1:length(PosOri)
    % Here uses to make sure the start singular configuration can go to the
    % selected first mode by adjusting q11 and q21
    if isempty(PosOri_current{i}) == 1  && i <= 6 
        Pos_Intep(i,:) = zeros(1,n);
        Vel_Intep(i,:) = zeros(1,n);
        Acc_Intep(i,:) = zeros(1,n);
    else        
        if i == 7
            if isempty(PosOri_current{i}) == 1
                if length(PosOri_previous) == 8 && isempty(PosOri_previous{i}) ~= 1
                    PosOri_current{i} = PosOri_previous{i};
                else
                    PosOri_previous{i} = 0;
                    PosOri_current{i} = 0;
                end
            else
                if (length(PosOri_previous) ~= 8 && (Mode ~= 10 && Mode ~= 11)) || (length(PosOri_previous) == 8 && isempty(PosOri_previous{i}) == 1)
                    PosOri_previous{i} = PosOri_current{i};
                end
            end
        elseif i == 8 
            if isempty(PosOri_current{i}) == 1
                if length(PosOri_previous) == 8 && isempty(PosOri_previous{i}) ~= 1
                    PosOri_current{i} = PosOri_previous{i};
                else
                    PosOri_previous{i} = 0;
                    PosOri_current{i} = 0;
                end
            else
                if (length(PosOri_previous) ~= 8 && (Mode ~= 10 && Mode ~= 11)) || (length(PosOri_previous) == 8 && isempty(PosOri_previous{i}) == 1)
                    PosOri_previous{i} = PosOri_current{i};
                end
            end
        end
        %po_Intep(i,:) = linspace(PosOri_previous{i},PosOri_current{i}, n);
       
        PO = [PosOri_previous{i},PosOri_current{i}];
        v = [0, 0];
        a = [0, 0];
        
        [ a0,a1,a2,a3,a4,a5,T ] =  PTP_Polynom5(PO, v, a, Time) ;
        
        t = linspace(0,T,n);
        px = [a5,a4,a3,a2,a1,a0];
        %px = [a0,a1,a2,a3,a4,a5];
        pxd = polyder(px);
        pxdd = polyder(pxd);
        Pos = polyval(px,t);
        Vel = polyval(pxd,t);
        Acc = polyval(pxdd,t);
        time_Intep = t + Time(1)*ones(1,n);
        
        Pos_Intep(i,:) = Pos;
        Vel_Intep(i,:) = Vel;
        Acc_Intep(i,:) = Acc;
    end
end

if Mode == 3
     Pos_Intep(1,:) = -l1/2 * sin(Pos_Intep(4,:));
     Pos_Intep(2,:) = l1/2 * (cos(Pos_Intep(4,:)) - 1);     
elseif Mode == 4     
     Pos_Intep(1,:) = l1/2 * sin(Pos_Intep(4,:));
     Pos_Intep(2,:) = l1/2 * (- cos(Pos_Intep(4,:)) + 1);    
end
%=============================== End ======================================

end
% 
% function [ PosVelAcc ] = Intepotation(p, v, a, t)
% 
% [ a0,a1,a2,a3,a4,a5,T ] = Polynom5(p, v, a, t) ;
% 
% t = linspace(0,T,100);
% px = [a5,a4,a3,a2,a1,a0];
% pxd = polyder(px);
% pxdd = polyder(pxd);
% Pos = polyval(px,t);
% Vel = polyval(pxd,t);
% Acc = polyval(pxdd,t);
% 
% end