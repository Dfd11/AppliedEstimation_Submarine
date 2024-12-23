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
lambda = alpha^2 * (n + kappa) - n; % Scaling factor
gamma = sqrt(n + lambda);

% Weights and sigma points
Wm = [lambda/(n+lambda), 0.5/(n+lambda)*ones(1,2*n)]; % 
Wc = Wm;
Wc(1) = Wc(1) + (1 - alpha^2 + beta);
S = chol(sigma,"lower"); % Lower trianquilar, square root of sigma Cholesky descomposition
X = [mu, mu + gamma*S, mu - gamma*S];

%% Prediction Step
for i=1:2*n+1
    X_pred(:,i)= predict_ukf(X(:,i), u,dt,drag);
end

x_pred = sum(Wm .* X_pred,2);
R_pred = R; 
for i=1:2*n+1
    R_pred = R_pred +  Wc(i) * (X_pred(:,i)-x_pred)*(X_pred(:,i)-x_pred)';
end

%% Update Step
Z_pred = zeros(m,2*n+1);
for i=1:2*n+1
    [Z_pred(:,i), Q_pred] = g_measurement(X_pred(:, i));
end

z_pred = sum(Wm .* Z_pred, 2);

% Innovation covariance
Q_zz = Q_pred;
for i = 1:2*n+1
    Q_zz = Q_zz + Wc(1)*(Z_pred(:,i) - z_pred) * (Z_pred(:,i) - z_pred)';
end

% Cross-covariance
P_xz = zeros(n,m);
for i = 1:2*n+1
    P_xz = P_xz + Wc(i) * (X_pred(:, i) - x_pred) * (Z_pred(:, i) - z_pred)';
end

% Kalman gain
K = P_xz/Q_zz;

% Update state and covariance
mu = x_pred + K * (z - z_pred);
sigma = R_pred - K * Q_zz * K';

end

