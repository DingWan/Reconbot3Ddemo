function [ po_Intep ] =  IntepotationP2P(Mode, PosOri_previous,q0q1q2_previous_trajpoint, PosOri_current, q0q1q2_current_trajpoint, NumIntepoPoints, l1, l2)

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
        po_Intep(i,:) = 0;
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
        po_Intep(i,:) = linspace(PosOri_previous{i},PosOri_current{i}, n);
    end
end

if Mode == 3
     po_Intep(1,:) = -l1/2 * sin(po_Intep(4,:));
     po_Intep(2,:) = l1/2 * (cos(po_Intep(4,:)) - 1);     
elseif Mode == 4     
     po_Intep(1,:) = l1/2 * sin(po_Intep(4,:));
     po_Intep(2,:) = l1/2 * (- cos(po_Intep(4,:)) + 1);    
end
%=============================== End ======================================


end