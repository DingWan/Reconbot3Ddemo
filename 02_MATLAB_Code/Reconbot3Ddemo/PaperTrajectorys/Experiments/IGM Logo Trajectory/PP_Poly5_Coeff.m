function [ coeff ] = PP_Poly5_Coeff( x_end, y_start,y_end,v_start,v_end,a_start,a_end )

    coeff=[a_end/(2*x_end^3) - a_start/(2*x_end^3) - (3*v_end)/x_end^4 - (3*v_start)/x_end^4 + (6*y_end)/x_end^5 - (6*y_start)/x_end^5;
           (3*a_start)/(2*x_end^2) - a_end/x_end^2 + (7*v_end)/x_end^3 + (8*v_start)/x_end^3 - (15*y_end)/x_end^4 + (15*y_start)/x_end^4;
            a_end/(2*x_end) - (3*a_start)/(2*x_end) - (4*v_end)/x_end^2 - (6*v_start)/x_end^2 + (10*y_end)/x_end^3 - (10*y_start)/x_end^3;
            a_start/2;
            v_start;
            y_start];
    
end

