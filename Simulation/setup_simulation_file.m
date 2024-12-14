function [simulation] = setup_simulation_file(root,imu_file,press_file,dvl_file, vbs_file,true_file)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
imu     =readtable(strcat(root,imu_file));
press   =readtable(strcat(root,press_file));
dvl     =readtable(strcat(root,dvl_file));
vbs     =readtable(strcat(root,vbs_file));
gtruth  =readtable(strcat(root,true_file));

%Convert from table to matrix
imu    = imu{:,:};
press  = press{:,:};
dvl    = dvl{:,:};
vbs    = vbs{:,:};
gtruth = gtruth{:,:};

%Go from ns to seconds
imu(:,1)    = imu(:,1)/1e9;
press(:,1)  = press(:,1)/1e9;
dvl(:,1)    = dvl(:,1)/1e9;
vbs(:,1)    = vbs(:,1)/1e9;
gtruth(:,1) = gtruth(:,1)/1e9;

%Get the smallest time start
start_imu       = min(imu(:,1));
start_press     = min(press(:,1));
start_dvl       = min(dvl(:,1));
start_vbs       = min(vbs(:,1));
start_gtruth    = min(gtruth(:,1));

min_start = min([start_imu start_press start_dvl start_vbs start_gtruth]);


%Offset all to the start value
imu(:,1)    = imu(:,1) - min_start;
press(:,1)  = press(:,1)- min_start;
dvl(:,1)    = dvl(:,1)- min_start;
vbs(:,1)    = vbs(:,1)- min_start;
gtruth(:,1) = gtruth(:,1)- min_start;

%For the creation of the index values
start_imu       = round(imu(1,1),1)/0.1;
start_press     = round(press(1,1),1)/0.1;
start_dvl       = round(dvl(1,1),1)/0.1;
start_vbs       = round(vbs(1,1),1)/0.1;
start_gtruth    = round(gtruth(1,1),1)/0.1;

imu(:,1)    = (0 : size(imu,1)-1 )' + start_imu;
press(:,1)  = (0 : size(press,1)-1 )' + start_press;
dvl(:,1)    = (0 : size(dvl,1)-1 )' + start_dvl;
vbs(:,1)    = (0 : size(vbs,1)-1 )' + start_vbs;
gtruth(:,1) = (0 : size(gtruth,1)-1 )' + start_gtruth;

%Simulation Matrix, its missing the adaptability for more columns and more
%rows
simulation = zeros(771,12); %DAVID Needs fixing rows can change as well as columns

%Time Stamp in seconds up to 1 decimal
simulation(:,1) = (0:size(simulation,1)-1)';
simulation(:,1) = simulation(:,1)*0.1;

%VBS control
simulation(int32(vbs(:,1)+1),2) = vbs(:,2);
%Ground Truth
simulation(int32(gtruth(:,1)+1),3) = gtruth(:,4);
%IMU Enable | Pressure Enable | DVL Enable
simulation(int32(imu(:,1)+1),4) = 1;
simulation(int32(dvl(:,1)+1),5) = 1;
simulation(int32(press(:,1)+1),6) = 1;
% IMU Z Accel | Water Pressure | DVL_X_vel | DVL_Y_vel | DVL_Z_vel | DVL_altitude
simulation(int32(imu(:,1)+1),7) = imu(:,2);
simulation(int32(dvl(:,1)+1),8:11) = dvl(:,2:5);
simulation(int32(press(:,1)+1),12) = press(:,2);


writematrix(simulation,strcat(root,'ocean_sim.txt'),'Delimiter','space');
end