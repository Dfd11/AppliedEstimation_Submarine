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
sigma = diag(100*ones(1,3));

R = [10 0 0;
     0 10 0;
     0 0 10
     ];

Q = [1 0;
     0 1
     ];

%% Constants
C2 = 101325.0 ; %From SAM ROS
C3 = 9806.65 ; %From SAM ROS
g= 9.81 ; %m/s^2
p=1000;
VSAM = 0.0202555; %m^3 %DAVID change SAM Volume
VVBS = 2.5535e-4 ; %m^3 %DAVID Check if we are still going to use this calculated value or ROS?
MSAM = 20; %kg %DAVID change SAM Mass

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
depth_lineHandle = plot(depth_ax, NaN, NaN, 'g-', 'DisplayName', 'Data');
true_lineHandle = plot(depth_ax, NaN, NaN, 'k-', 'DisplayName', 'Data');

errorBarHandle = errorbar(depth_ax, NaN, NaN, NaN, 'o', 'Color', [0, 0.447, 0.741], 'DisplayName', 'Uncertainty');


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
error_lineHandle = plot(error_ax, NaN, NaN, '-o', 'DisplayName', 'Data');

 
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
        z = [z ; sim_data(tstep,12)];
    end
    
    %Predict Phase mu = mu(t-1) + u
    u = calculate_odometry(VSAM,VVBS,MSAM,g,vbs,dt,mu);
    [mu_bar, sigma_bar] = predict_(mu, sigma, u,dt);

    %Update Phase
    try
        [mu, sigma] = update_(mu_bar, sigma_bar,C3, C2,z);
    catch
        warning("Error")
        pose_errors(tstep) = true_pose - mu(3); %Error for the depth
        sigmas(:,tstep) = sigma(:); %Covariance matrix
        timesteps(tstep) = t; %Time stamp
        break;
    end

    % Statistics
    state(:,tstep) = mu(:);
    pose_errors(tstep) = true_pose - mu(3); %Error for the depth
    sigmas(:,tstep) = sigma(:); %Covariance matrix
    timesteps(tstep) = t; %Time stamp

    set(depth_lineHandle,'XData',timesteps(1:tstep),'YData',state(3,1:tstep));
    set(true_lineHandle,'XData',timesteps(1:tstep),'YData',sim_data(1:tstep,3));
    set(errorBarHandle,'XData',timesteps(tstep),'YData',state(3,tstep),'UData',sqrt(sigmas(9,tstep)),'LData',sqrt(sigmas(9,tstep)));
    title(depth_ax, [depth_graphTitle, ' - Timestep ', num2str(t)]);
    title(error_ax, [error_graphTitle, ' - Timestep ', num2str(t)]);
    pause(0.1);
end

