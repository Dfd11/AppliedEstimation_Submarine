function [acc_min,acc_max,vel_min,vel_max,depth_min,depth_max] = calc_limits_sigma(sigmas,acc_min,acc_max,vel_min,vel_max,depth_min,depth_max)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if sigmas(1) > acc_max
    acc_max = sigmas(1) + 1;
end

if sigmas(1) < acc_min
    acc_min = sigmas(1) - 1;
end

if sigmas(5) > vel_max
    vel_max = sigmas(5) + 1;
end

if sigmas(5) < vel_min
    vel_min = sigmas(5) - 1;
end

if sigmas(9) > depth_max
    depth_max = sigmas(9) + 1;
end

if sigmas(9) < depth_min
    depth_min = sigmas(9) - 1;
end



end