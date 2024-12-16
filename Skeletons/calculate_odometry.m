% This function calculates the odometry information.
function u = calculate_odometry(VSAM,VVBS,MSAM,g,vbs,delta_t, mu)
    if ~delta_t
        u = [0;0;0];
        return;
    end

    % YOUR IMPLEMENTATION %
    p = 1000; %kg/m^3 constant for now %DAVID are we changing for depth varying?
    %u = [ p*(VSAM-VVBS*vbs/100)*g/MSAM - g  ;
    %      0  ;
    %      0  ];
    %u = [p*g*VSAM / (MSAM + (p*VVBS*vbs/100)) - g ;
    %     0;
    %     0];
    u = [(p*g*VSAM - MSAM*g - p*VVBS*g*vbs/100)/(MSAM + p*VVBS*vbs/100)
         0 ;
         0 ];
    return;


end