% This function should perform the update process(sequential update).
% You need to make sure that the output sigma_bar is symmetric.
% The last line makes sure that ouput sigma_bar is always symmetric.
% Inputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
%           H_bar(t)        2X3
%           S_bar(t)        2X2
%           nu_bar(t)       2X1
% Outputs:
%           mu_bar(t)       3X1
%           sigma_bar(t)    3X3
function z_hat = g_measurement(mu_bar)
        global Q
        global meas_en
        % YOUR IMPLEMENTATION %
        
        h = [];
        H = [];
        if meas_en(2)
            h = [h ; mu_bar(2)];
            H = [H ; 0 1 0];
        end

        if meas_en(3)
%            h = [h ; -mu_bar(3)*C3 + C2]; %The other option is to just put mu_bar(3) here and when calculating the innovation do it in terms of depth and not water pressure
            h = [h ; mu_bar(3)];
%            H = [H ; 0 0 -C3]; %DAVID change everything to be done in terms of m and not in terms of pascals
            H = [H ; 0 0 1];
        end
        
        %% Noise Matrix
        if meas_en(2) && meas_en(3) %Both DVL and Pressure
            temp_q = Q;
        elseif meas_en(2) %Only DVL
            temp_q = Q(1,1); 
        elseif meas_en(3) %Only Pressure
            temp_q =Q(2,2);
        else
            temp_q = NaN; %If none available
        end

        z_hat = h;

end
