% This function performs the prediction step.
% Inputs:
%           mu(t-1)           3X1   
%           sigma(t-1)        3X3
%           u(t)              3X1
% Outputs:   
%           mu_bar(t)         3X1
%           sigma_bar(t)      3X3
function mu_bar = predict_ukf(mu, u,delta_t,drag)
    %DAVID this is where the state is updated, and how the covariance
    %matrix is updated too by the uncertainty of the motion or prediction
    %model in here we will add the noise based on how our model is not
    %correctly predicting, i.e. we predict a linear motion model but in
    %reality is not
    global R % covariance matrix of motion model | shape 3X3

    % YOUR IMPLEMENTATION %
    %mu_bar = [  u(1);
    %            delta_t*mu(1) + mu(2);
    %            1/2*delta_t^2*mu(1) + delta_t*mu(2) + mu(3)
    %         ];
    mu_bar = [  -drag*sign(mu(2))*mu(2)^2 + u(1);
                delta_t*mu(1) + mu(2);
                1/2*delta_t^2*mu(1) + delta_t*mu(2) + mu(3)
             ];

end
