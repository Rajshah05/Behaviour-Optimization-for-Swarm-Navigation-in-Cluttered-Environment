clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Initial Control gains:
k_ria = 20;  %(inter-agent position)
k_via = .26;  %(inter-agent velocities)
k_rvl = .5; %(virtual-leader position)
k_vvl = .25;  %(virtual-leader velocity)
k_obs = 4;   %(obstacle position)
gains = [k_ria,k_via,k_rvl,k_vvl,k_obs];

% [ts,error_history] = simulate(gains);
% error_history(error_history == 0) = [];
% plot(error_history)

% Optimize:
x = fmincon(@simulate, gains, [],[]);