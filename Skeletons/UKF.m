function [mu, sigma] = UKF(mu, sigma, u, z,dt,drag)
% UKF - Unscented Kalman Filter algorithm
% mu    - Prior mean                      3X1
% Sigma - Prior covariance                3X3
% u     - Control input                   3X1
% z     - Measurement                                       
% R     - Process noise covariance        3X3
% Q     - Measurement noise covariance
% alpha, beta, kappa - UKF parameters

global Q;
global R;

alpha = 1e-3; % Scale parameter
kappa = 0;    % Second scaling parameter
beta  = 2;    % Optimal for Gaussian distributions 

n = length(mu); % Number of states: 3
m = length(z);  % Number of measurements
%m = size(R, 1); % Number of process noise components: 3
%N = n + m;      % Augmented state size: 6
lambda = alpha^2 * (n + kappa) - n; % Scaling factor underneath of 7.30
gamma = sqrt(n + lambda); % Bottom of Table 7.3.2

% Weights and sigma points
Wm = [lambda/(n+lambda), 0.5/(n+lambda)*ones(1,2*n)]; % 7.34
Wc = Wm; % 7.34
Wc(1) = Wc(1) + (1 - alpha^2 + beta); % 7.34
try
S = chol(sigma,"lower"); % Lower trianquilar, square root of sigma Cholesky descomposition, variations for numerical purposes in Implementation Variations
catch
    warning("Error")
    return;
end
X = [mu, mu + gamma*S, mu - gamma*S]; %(7.52)

%% Prediction Step
for i=1:2*n+1
    X_pred(:,i)= predict_ukf(X(:,i), u,dt,drag); % 7.40
end

x_pred = sum(Wm .* X_pred,2); % 7.41
R_pred = R; 
for i=1:2*n+1
    R_pred = R_pred +  Wc(i) * (X_pred(:,i)-x_pred)*(X_pred(:,i)-x_pred)'; % 7.42
end

%% Update Step
if ~isempty(z)
    Z_pred = zeros(m,2*n+1);
    for i=1:2*n+1
        [Z_pred(:,i), Q_pred] = g_measurement(X_pred(:, i)); % 7.43
    end
    
    z_pred = sum(Wm .* Z_pred, 2); % 7.44
    
    % Innovation covariance
    Q_zz = Q_pred;
    for i = 1:2*n+1
        Q_zz = Q_zz + Wc(1)*(Z_pred(:,i) - z_pred) * (Z_pred(:,i) - z_pred)'; % 7.45
    end
    
    % Cross-covariance
    P_xz = zeros(n,m);
    for i = 1:2*n+1
        P_xz = P_xz + Wc(i) * (X_pred(:, i) - x_pred) * (Z_pred(:, i) - z_pred)'; % 7.46
    end
    
    % Kalman gain
    K = P_xz/Q_zz; % 7.47
    
    % Update state and covariance
    mu = x_pred + K * (z - z_pred); % 7.48
    sigma = R_pred - K * Q_zz * K'; % 7.49
else
    mu = x_pred ;
    sigma = R_pred ;
end
end

