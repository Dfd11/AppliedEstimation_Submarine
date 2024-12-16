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
%mu_bar =[0 ;0 ;0];
%sigma = diag(100*ones(1,3));
%sigma_bar = zeros(3,3);

R = [0.001^2 0 0;
     0 0.0005^2 0;
     0 0 0.001^2
     ];
sigma = R;
Q = [0.15^2 0; %This noise has to be in terms of m/s
     0 0.15^2 %This noise has to be in terms of pascals
     ];

%% Constants
C2 = 101325.0 ; %1 Atmosphere in Pascals
C3 = 9806.65 ; % g times the water density p
g= 9.80665 ; %m/s^2
p=1000;
VVBS = 2.5535e-4 ; %m^3 %DAVID Check if we are still going to use this calculated value or ROS?
MSAM = 250; %kg %DAVID change SAM Mass
%VSAM = 0.020127675; %m^3 %DAVID change SAM Volume
VSAM = (MSAM*g+0.75)/(g*p); %DAVID this was estimated based on the fact that the volume of VBS was about a 2.5 N range, so displaced water volume and SAM weigth should be within that range

%% Sim Initialize


%t = 0;
dt = 0.1;
n_timesteps = size(sim_data,1);
global meas_en

%Storage Arrays
state = zeros(3,n_timesteps);
timesteps = zeros(1, n_timesteps);
pose_errors = zeros(1, n_timesteps);
sigmas = zeros(size(sigma(:), 1), n_timesteps);
sensor_depth = zeros(1,n_timesteps);

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

% Data storage
%xData = [];
%yData = [];
%stdDevs = [];
depth_lineHandle = plot(depth_ax, NaN, NaN, 'g-', 'DisplayName', 'Estimated Depth');
true_lineHandle = plot(depth_ax, NaN, NaN, 'k-', 'DisplayName', 'True Depth');
press_lineHandle = scatter(depth_ax,NaN,NaN,3,"blue","filled",'DisplayName','Press Sensor','MarkerFaceAlpha',3/8);

errorBarHandle = errorbar(depth_ax, NaN, NaN, NaN, '.', 'Color', [0, 0.447, 0.741], 'DisplayName', 'Uncertainty');

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
error_lineHandle = plot(error_ax, NaN, NaN, 'k-', 'DisplayName', 'Data');
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
depth_sigma_min = -1;
depth_sigma_max = 1;

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
accel_sigma_lineHandle = plot(accel_sigma, NaN, NaN, 'k-', 'DisplayName', 'Accel Cov');
vel_sigma_lineHandle = plot(vel_sigma, NaN, NaN, 'k-', 'DisplayName', 'Vel Cov');
depth_sigma_lineHandle = plot(depth_sigma, NaN, NaN, 'k-', 'DisplayName', 'Depth Cov');

 
%% EKF Loop
for tstep=1:n_timesteps
    t = sim_data(tstep,1);
    vbs = sim_data(tstep,2);
    true_pose = sim_data(tstep,3);
    meas_en = sim_data(tstep,4:6);

    z = [];
    if meas_en(2) %DVL
        z = [z ; sim_data(tstep,10)];
    end

    if meas_en(3) %Pressure
%        single_sens_reading = sim_data(tstep,12)+ sqrt(Q(2,2))*randn;
        single_sens_reading = -(sim_data(tstep,12)-C2)/C3 + sqrt(Q(2,2))*randn;
        z = [z ; single_sens_reading];
    end
    
    %Predict Phase mu = mu(t-1) + u
    u = calculate_odometry(VSAM,VVBS,MSAM,g,vbs,dt,mu);
    [mu, sigma] = predict_(mu, sigma, u,dt);

    %Update Phase
    try
        [mu, sigma] = update_(mu, sigma,C3, C2,z);
    catch
        warning("Error")
        pose_errors(tstep) = true_pose - mu(3); %Error for the depth
        sigmas(:,tstep) = sigma(:); %Covariance matrix
        timesteps(tstep) = t; %Time stamp
        break;
    end

    % Statistics
    state(:,tstep) = mu(:);
    pose_errors(tstep) = abs(true_pose - mu(3)); %Error for the depth
%    sensor_depth(tstep) = -(single_sens_reading-C2)/C3; %Sensor reading converted into negative depth
    sensor_depth(tstep) = single_sens_reading;
    sigmas(:,tstep) = sigma(:); %Covariance matrix
    timesteps(tstep) = t; %Time stamp

    % Plots
    set(depth_lineHandle,'XData',timesteps(1:tstep),'YData',state(3,1:tstep)); %Estimated State
    set(true_lineHandle,'XData',timesteps(1:tstep),'YData',sim_data(1:tstep,3)); %True State
    set(press_lineHandle,'XData',timesteps(1:tstep),'YData',sensor_depth(1:tstep)); %Sensor Measurement With noise
    set(errorBarHandle,'XData',timesteps(tstep),'YData',state(3,tstep),'UData',sqrt(sigmas(9,tstep)),'LData',sqrt(sigmas(9,tstep)));

    set(error_lineHandle,'XData',timesteps(1:tstep),'YData',pose_errors(1:tstep));

    [accel_sigma_min,accel_sigma_max,vel_sigma_min,vel_sigma_max,depth_sigma_min,depth_sigma_max] = calc_limits_sigma(sigmas(:,tstep),accel_sigma_min,accel_sigma_max,vel_sigma_min,vel_sigma_max,depth_sigma_min,depth_sigma_max);
    accel_sigma.YLim = [accel_sigma_min , accel_sigma_max];
    vel_sigma.YLim = [vel_sigma_min , vel_sigma_max];
    depth_sigma.YLim = [depth_sigma_min , depth_sigma_max];
    set(accel_sigma_lineHandle,'XData',timesteps(1:tstep),'YData',sigmas(1,1:tstep));
    set(vel_sigma_lineHandle,'XData',timesteps(1:tstep),'YData',sigmas(5,1:tstep));
    set(depth_sigma_lineHandle,'XData',timesteps(1:tstep),'YData',sigmas(9,1:tstep));
    


    title(depth_ax, [depth_graphTitle, ' - Timestep ', num2str(t)]);
    title(error_ax, [error_graphTitle, ' - Timestep ', num2str(t)]);
    pause(0.1);
end

