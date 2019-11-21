clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Initial Control gains:
k_ria = 20;  %(inter-agent position)
k_via = .25;  %(inter-agent velocities)
k_rvl = .5; %(virtual-leader position)
k_vvl = .25;  %(virtual-leader velocity)
k_obs = 2;   %(obstacle position)
gains = [k_ria,k_via,k_rvl,k_vvl,k_obs];

%% Perform Optimization:
num_runs = 1;
for ii = 1:num_runs
    % Simulate the model:
    ts = simulate(gains,1);
    
    % Run the optimization scheme:
    
end