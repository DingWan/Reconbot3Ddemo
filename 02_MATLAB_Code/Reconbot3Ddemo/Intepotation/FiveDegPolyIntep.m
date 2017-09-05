%% 5-Grade Polynomial Intepotation
function [Pos_Intep, Vel_Intep, Acc_Intep] = FiveDegPolyIntep(PreviousValue, CurrentValue, n, Time)
        PO = [PreviousValue,CurrentValue];
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
        
        Pos_Intep(:) = Pos;
        Vel_Intep(:) = Vel;
        Acc_Intep(:) = Acc;
        
end