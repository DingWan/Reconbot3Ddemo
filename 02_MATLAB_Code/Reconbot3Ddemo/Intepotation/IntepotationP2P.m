function [ po_Intep ] =  IntepotationP2P(Mode, PosOri_previous, PosOri_current, NumIntepoPoint)

%% =================== Catisian Space Trajctory Planning ====================
PosOri = PosOri_current;
n = NumIntepoPoint;
for i = 1:length(PosOri)
    % Here uses to make sure the start singular configuration can go to the
    % selected first mode by adjusting q11 and q21
    if isempty(PosOri{i}) == 1
        po_Intep(i,:) = 0;
    else
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