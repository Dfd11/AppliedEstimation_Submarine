%% File Setup
simulationfile = 'ocean_sim.txt';
imu_file = 'imu.csv';
press_file = 'fluid_pressure.csv';
dvl_file = 'dvl.csv';
true_file = 'odom.csv';
vbs_file = 'vbs.csv';

sim_data = setup_simulation_file('DataSets/',imu_file,press_file,dvl_file,vbs_file,true_file);


%% Initialize EKF Parameters
global R 
global Q 
mu = [0 ; 0 ; 0];
mu_ukf = mu;
%mu_bar =[0 ;0 ;0];
%sigma = diag(100*ones(1,3));
%sigma_bar = zeros(3,3);
EKF_UKF = 2;
R = [0.001^2 0 0;
     0 0.008^2 0;
     0 0 0.004^2
     ];
sigma = R;
sigma_ukf = sigma;
Q = [0.05^2 0; %This noise has to be in terms of m/s
     0 0.15^2 %This noise has to be in terms of pascals
     ];
DVL_freq = 0.1;
press_freq = 0.1;
%% Constants
C2 = 101325.0 ; %1 Atmosphere in Pascals
C3 = 9806.65 ; % g times the water density p
g= 9.80665 ; %m/s^2
p=1000;
VVBS = 2.5535e-4 ; %m^3 %DAVID Check if we are still going to use this calculated value or ROS?
%MSAM = 250; %kg %DAVID change SAM Mass
MSAM = 12.012 + 0.3;
%VSAM = 0.020127675; %m^3 %DAVID change SAM Volume
%VSAM = (MSAM*g+0.75)/(g*p); %DAVID this was estimated based on the fact that the volume of VBS was about a 2.5 N range, so displaced water volume and SAM weigth should be within that range
VSAM = (MSAM*g+1.25)/(g*p);
drag = 12.5;


if EKF_UKF ==0 | EKF_UKF == 2
    disp('Running EKF')
end

if EKF_UKF ==1 | EKF_UKF == 2
    disp('Running UKF')
end

%% Sim Initialize

%t = 0;
dt = 0.1;
dvl_count = DVL_freq/dt;
press_count = press_freq/dt;
n_timesteps = size(sim_data,1);
global meas_en

%Storage Arrays
state = zeros(3,n_timesteps);
pos_state = zeros(1,n_timesteps);
neg_state = zeros(1,n_timesteps);
timesteps = zeros(1, n_timesteps);
pose_errors = zeros(1, n_timesteps);
sigmas = zeros(size(sigma(:), 1), n_timesteps);
sensor_depth = zeros(1,n_timesteps);

state_ukf = zeros(3,n_timesteps);
pos_state_ukf = zeros(1,n_timesteps);
neg_state_ukf = zeros(1,n_timesteps);
pose_errors_ukf = zeros(1, n_timesteps);
sigmas_ukf = zeros(size(sigma_ukf(:), 1), n_timesteps);

%% Depth Plot
%Params
t_margin = 0.5;
% Initialize figure and settings
depth_fig = figure('Name', 'Depth', 'NumberTitle', 'off');
depth_ax = axes(depth_fig);
hold(depth_ax, 'on');

% Default graph settings
depth_graphTitle = 'Depth';
xLimits = [-t_margin, n_timesteps*dt+t_margin];
yLimits = [-3.5 , 0.1];
gridOn = true;

% Configure plot
depth_ax.XLim = xLimits;
depth_ax.YLim = yLimits;
grid(depth_ax, gridOn);
title(depth_ax, depth_graphTitle);
xlabel(depth_ax, 'Timesteps');
ylabel(depth_ax, 'Depth');

% Initialize legend
legend(depth_ax, 'Location', 'best');
if EKF_UKF ==0 | EKF_UKF == 2
depth_lineHandle = plot(depth_ax, NaN, NaN, 'g-', 'DisplayName', 'Estimated Depth EKF');
pos_lineHandle = plot(depth_ax, NaN, NaN, 'r-','DisplayName', 'Pos Std Dev EKF');
neg_lineHandle = plot(depth_ax, NaN, NaN, 'r-','DisplayName', 'Neg Std Dev EKF');
end
if EKF_UKF ==1 | EKF_UKF == 2
depth_lineHandle_ukf = plot(depth_ax, NaN, NaN, 'm-', 'DisplayName', 'Estimated Depth UKF');
pos_lineHandle_ukf = plot(depth_ax, NaN, NaN, 'c-','DisplayName', 'Pos Std Dev UKF');
neg_lineHandle_ukf = plot(depth_ax, NaN, NaN, 'c-','DisplayName', 'Neg Std Dev UKF');
end
true_lineHandle = plot(depth_ax, NaN, NaN, 'k-', 'DisplayName', 'True Depth');
press_lineHandle = scatter(depth_ax,NaN,NaN,3,"blue","filled",'DisplayName','Press Sensor','MarkerFaceAlpha',3/8);


%% Error Plot
%Params
t_margin = 0.5;
% Initialize figure and settings
error_fig = figure('Name', 'Error', 'NumberTitle', 'off');
error_ax = axes(error_fig);
hold(error_ax, 'on');

% Default graph settings
error_graphTitle = 'Depth Error';
xLimits = [-t_margin, n_timesteps*dt+t_margin];
yLimits = [min(state(3,:)) - 0.5 , max(state(3,:)) + 0.5];
gridOn = true;
showErrorBars = true;

% Configure plot
error_ax.XLim = xLimits;
error_ax.YLim = yLimits;
grid(error_ax, gridOn);
title(error_ax, error_graphTitle);
xlabel(error_ax, 'Timesteps');
ylabel(error_ax, 'Error');

% Initialize legend
legend(error_ax, 'Location', 'best');

% Data storage
%xData = [];
%yData = [];
%stdDevs = [];
if EKF_UKF ==0 | EKF_UKF == 2
error_lineHandle = plot(error_ax, NaN, NaN, 'g-', 'DisplayName', 'Error EKF');
end
if EKF_UKF ==1 | EKF_UKF == 2
error_lineHandle_ukf = plot(error_ax, NaN, NaN, 'm-', 'DisplayName', 'Error UKF');
end
%% Sigmas Plot
%Params
t_margin = 0.5;
% Initialize figure and settings
sigma_fig = figure('Name', 'Sigma', 'NumberTitle', 'off');

accel_sigma = subplot(3, 1, 1, 'Parent', sigma_fig);
vel_sigma = subplot(3, 1, 2, 'Parent', sigma_fig);
depth_sigma = subplot(3, 1, 3, 'Parent', sigma_fig);

hold(accel_sigma, 'on');
hold(vel_sigma, 'on');
hold(depth_sigma, 'on');

% Default graph settings
sigma_graphTitle = 'Covariance';

accel_sigma_min = -1;
accel_sigma_max = 5;
vel_sigma_min = -1;
vel_sigma_max = 5;
depth_sigma_min = -0.1;
depth_sigma_max = 0.1;

% Configure plot
accel_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
accel_sigma.YLim = [accel_sigma_min , accel_sigma_max];
vel_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
vel_sigma.YLim = [vel_sigma_min , vel_sigma_max];
depth_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
depth_sigma.YLim = [depth_sigma_min , depth_sigma_max];
grid(accel_sigma, true);
grid(vel_sigma, true);
grid(depth_sigma, true);
title(accel_sigma, strcat(sigma_graphTitle,' Accel'));
title(vel_sigma, strcat(sigma_graphTitle,' Vel'));
title(depth_sigma, strcat(sigma_graphTitle,' Depth'));
xlabel(accel_sigma, 'Timesteps');
ylabel(accel_sigma, 'Variance');

% Initialize legend
legend(accel_sigma, 'Location', 'best');
legend(vel_sigma, 'Location', 'best');
legend(depth_sigma, 'Location', 'best');
% Data storage
%xData = [];
%yData = [];
%stdDevs = [];
if EKF_UKF ==0 | EKF_UKF == 2
accel_sigma_lineHandle = plot(accel_sigma, NaN, NaN, 'g-', 'DisplayName', 'Accel Cov EKF');
vel_sigma_lineHandle = plot(vel_sigma, NaN, NaN, 'g-', 'DisplayName', 'Vel Cov EKF');
depth_sigma_lineHandle = plot(depth_sigma, NaN, NaN, 'g-', 'DisplayName', 'Depth Cov EKF');
end
if EKF_UKF ==1 | EKF_UKF == 2
accel_sigma_lineHandle_ukf = plot(accel_sigma, NaN, NaN, 'm-', 'DisplayName', 'Accel Cov UKF');
vel_sigma_lineHandle_ukf = plot(vel_sigma, NaN, NaN, 'm-', 'DisplayName', 'Vel Cov UKF');
depth_sigma_lineHandle_ukf = plot(depth_sigma, NaN, NaN, 'm-', 'DisplayName', 'Depth Cov UKF');
end

%% Loop
for tstep=1:n_timesteps
    t = sim_data(tstep,1);
    vbs = sim_data(tstep,2);
    true_pose = sim_data(tstep,3);
    meas_en = sim_data(tstep,4:6);

    z = [];
    if meas_en(2) %DVL
        if dvl_count == DVL_freq/dt;
            z = [z ; sim_data(tstep,10)];
            dvl_count = 1;
            meas_en(2) = 1;
        else
            dvl_count = dvl_count + 1;
            meas_en(2) = 0;
        end
    end

    if meas_en(3) %Pressure
        single_sens_reading = -(sim_data(tstep,12)-C2)/C3 + sqrt(0.15^2)*randn;
        if press_count == press_freq/dt;
    %        single_sens_reading = sim_data(tstep,12)+ sqrt(Q(2,2))*randn;

            z = [z ; single_sens_reading];
            press_count = 1;
            meas_en(3) = 1;
        else
            press_count = press_count + 1;
            meas_en(3) = 0;
        end
    end
    
    %Predict Phase mu = mu(t-1) + u
    u = calculate_odometry(VSAM,VVBS,MSAM,g,vbs,dt,mu);

    if EKF_UKF ==0 | EKF_UKF == 2
        % Predict phase
        [mu, sigma] = predict_(mu, sigma, u,dt,drag);
        % Update Phase
        try
            if ~isempty(z)
                [mu, sigma] = update_(mu, sigma,C3, C2,z);
            end
        catch
            warning("Error")
            pose_errors(tstep) = true_pose - mu(3); %Error for the depth
            sigmas(:,tstep) = sigma(:); %Covariance matrix
            timesteps(tstep) = t; %Time stamp
            break;
        end
    end
    if EKF_UKF ==1 | EKF_UKF == 2
        [mu_ukf, sigma_ukf] = UKF(mu_ukf, sigma_ukf, u, z,dt,drag);
    end

    % Statistics
    state(:,tstep) = mu(:);
    state_ukf(:,tstep) = mu_ukf(:);
   
    pose_errors(tstep) = (true_pose - mu(3)); %Error for the depth
    pose_errors_ukf(tstep) = (true_pose - mu_ukf(3)); %Error for the depth

    sigmas(:,tstep) = sigma(:); %Covariance matrix
    sigmas_ukf(:,tstep) = sigma_ukf(:); %Covariance matrix
    
    sensor_depth(tstep) = single_sens_reading;
    timesteps(tstep) = t; %Time stamp

end

%% Depth Plot

%EFK
if EKF_UKF ==0 | EKF_UKF == 2
pos_state = state(3,:) + sqrt(sigmas(9,:));
neg_state = state(3,:) - sqrt(sigmas(9,:));
set(depth_lineHandle,'XData',timesteps,'YData',state(3,:)); %Estimated State
set(pos_lineHandle,'XData',timesteps,'YData',pos_state); %Positive Std Dev
set(neg_lineHandle,'XData',timesteps,'YData',neg_state); %Negative Std Dev
set(error_lineHandle,'XData',timesteps,'YData',pose_errors(:));
end
if EKF_UKF ==1 | EKF_UKF == 2
%UKF
pos_state_ukf = state_ukf(3,:) + sqrt(sigmas_ukf(9,:));
neg_state_ukf = state_ukf(3,:) - sqrt(sigmas_ukf(9,:));
set(depth_lineHandle_ukf,'XData',timesteps,'YData',state_ukf(3,:)); %Estimated State
set(pos_lineHandle_ukf,'XData',timesteps,'YData',pos_state_ukf); %Positive Std Dev
set(neg_lineHandle_ukf,'XData',timesteps,'YData',neg_state_ukf); %Negative Std Dev
set(error_lineHandle_ukf,'XData',timesteps,'YData',pose_errors_ukf(:));
end
set(true_lineHandle,'XData',timesteps,'YData',sim_data(:,3)); %True State
set(press_lineHandle,'XData',timesteps,'YData',sensor_depth(:)); %Sensor Measurement With noise
title(depth_ax, [depth_graphTitle, ' - Timestep ', num2str(t)]);

%% State Plot
%Params
t_margin = 0.5;
% Initialize figure and settings
states_fig = figure('Name', 'States', 'NumberTitle', 'off');

accel = subplot(3, 1, 1, 'Parent', states_fig);
vel = subplot(3, 1, 2, 'Parent', states_fig);
depth = subplot(3, 1, 3, 'Parent', states_fig);

hold(accel, 'on');
hold(vel, 'on');
hold(depth, 'on');

% Default graph settings
states_graphTitle = 'State';


grid(accel, true);
grid(vel, true);
grid(depth, true);
title(accel, strcat(states_graphTitle,' Accel'));
title(vel, strcat(states_graphTitle,' Vel'));
title(depth, strcat(states_graphTitle,' Depth'));
xlabel(accel, 'Timesteps');
ylabel(accel, 'Variance');

% Initialize legend
legend(accel, 'Location', 'best');
legend(vel, 'Location', 'best');
legend(depth, 'Location', 'best');
% Data storage
%xData = [];
%yData = [];
%stdDevs = [];
%EKF
if EKF_UKF ==0 | EKF_UKF == 2
accel_lineHandle = plot(accel, NaN, NaN, 'g-', 'DisplayName', 'Est. Accel EKF');
vel_lineHandle = plot(vel, NaN, NaN, 'g-', 'DisplayName', 'Est. Vel EKF');
depth_lineHandle = plot(depth, NaN, NaN, 'g-', 'DisplayName', 'Est. Depth EKF');
end
accel_state = state(1,:);
vel_state = state(2,:);
depth_state = state(3,:);

%UKF
if EKF_UKF ==1 | EKF_UKF == 2
accel_lineHandle_ukf = plot(accel, NaN, NaN, 'm-', 'DisplayName', 'Est. Accel UKF');
vel_lineHandle_ukf = plot(vel, NaN, NaN, 'm-', 'DisplayName', 'Est. Vel UKF');
depth_lineHandle_ukf = plot(depth, NaN, NaN, 'm-', 'DisplayName', 'Est. Depth UKF');
end
accel_state_ukf = state_ukf(1,:);
vel_state_ukf = state_ukf(2,:);
depth_state_ukf = state_ukf(3,:);

true_accel_lineHandle = plot(accel, NaN, NaN, 'k-', 'DisplayName', 'True Accel');
true_vel_lineHandle = plot(vel, NaN, NaN, 'k-', 'DisplayName', 'True Vel');
true_depth_lineHandle = plot(depth, NaN, NaN, 'k-', 'DisplayName', 'True Depth');
true_vel = (sim_data(:,3)' - [0 sim_data(1:end-1,3)'])/dt;
true_accel = (true_vel - [0 true_vel(1:end-1)])/dt;

%% Configure state plot
accel_min = min([accel_state true_accel accel_state_ukf])*1.1;
accel_max = max([accel_state true_accel accel_state_ukf])*1.1;
vel_min = min([vel_state true_vel vel_state_ukf])*1.1;
vel_max = max([vel_state true_vel vel_state_ukf])*1.1;
depth_min = min([depth_state sim_data(:,3)' depth_state_ukf])*1.1;
depth_max = max([depth_state sim_data(:,3)' depth_state_ukf])*1.1;


accel.XLim = [-t_margin, n_timesteps*dt+t_margin];
accel.YLim = [accel_min , accel_max];
vel.XLim = [-t_margin, n_timesteps*dt+t_margin];
vel.YLim = [vel_min , vel_max];
depth.XLim = [-t_margin, n_timesteps*dt+t_margin];
depth.YLim = [depth_min , depth_max];
if EKF_UKF ==0 | EKF_UKF == 2
set(accel_lineHandle,'XData',timesteps,'YData',accel_state); %Positive Std Dev
set(vel_lineHandle,'XData',timesteps,'YData',vel_state); %Positive Std Dev
set(depth_lineHandle,'XData',timesteps,'YData',depth_state); %Positive Std Dev
end
if EKF_UKF ==1 | EKF_UKF == 2
set(accel_lineHandle_ukf,'XData',timesteps,'YData',accel_state_ukf); %Positive Std Dev
set(vel_lineHandle_ukf,'XData',timesteps,'YData',vel_state_ukf); %Positive Std Dev
set(depth_lineHandle_ukf,'XData',timesteps,'YData',depth_state_ukf); %Positive Std Dev
end
set(true_accel_lineHandle,'XData',timesteps,'YData',true_accel); %Positive Std Dev
set(true_vel_lineHandle,'XData',timesteps,'YData',true_vel); %Positive Std Dev
set(true_depth_lineHandle,'XData',timesteps,'YData',sim_data(:,3)'); %Positive Std Dev
  
%% Configure sigma plot
accel_min = min([sigmas(1,:) sigmas_ukf(1,:)] )*1.1;
accel_max = max([sigmas(1,:) sigmas_ukf(1,:)] )*1.1;
vel_min = min([sigmas(5,:) sigmas_ukf(5,:)] )*1.1;
vel_max = max([sigmas(5,:) sigmas_ukf(5,:)] )*1.1;
depth_min = min([sigmas(9,:) sigmas_ukf(9,:)] )*1.1;
depth_max = max([sigmas(9,:) sigmas_ukf(9,:)] )*1.1;


accel_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
accel_sigma.YLim = [accel_min , accel_max];
vel_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
vel_sigma.YLim = [vel_min , vel_max];
depth_sigma.XLim = [-t_margin, n_timesteps*dt+t_margin];
depth_sigma.YLim = [depth_min , depth_max];
if EKF_UKF ==0 | EKF_UKF == 2
set(accel_sigma_lineHandle,'XData',timesteps,'YData',sigmas(1,:));
set(vel_sigma_lineHandle,'XData',timesteps,'YData',sigmas(5,:));
set(depth_sigma_lineHandle,'XData',timesteps,'YData',sigmas(9,:));
end
if EKF_UKF ==1 | EKF_UKF == 2
set(accel_sigma_lineHandle_ukf,'XData',timesteps,'YData',sigmas_ukf(1,:));
set(vel_sigma_lineHandle_ukf,'XData',timesteps,'YData',sigmas_ukf(5,:));
set(depth_sigma_lineHandle_ukf,'XData',timesteps,'YData',sigmas_ukf(9,:));
end
%% Configure error plot
title(error_ax, [error_graphTitle, ' - Timestep ', num2str(t), ' - MSE EKF ', num2str(mean(pose_errors(1:tstep).^2)), ' - MSE UKF ', num2str(mean(pose_errors_ukf(1:tstep).^2))]);
