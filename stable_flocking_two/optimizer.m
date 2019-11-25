clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Initial Control gains:
k_ria = 20;  %(inter-agent position)
k_via = .25;  %(inter-agent velocities)
k_rvl = .5; %(virtual-leader position)
k_vvl = .25;  %(virtual-leader velocity)
k_obs = 30;   %(obstacle position)
obs_dist = 40;
gains = [k_ria,k_via,k_rvl,k_vvl,k_obs,obs_dist]';

% Optimize:
options = optimoptions('fmincon','FiniteDifferenceStepSize',1e-1);
% A = [4 0 0 0 30/31.5];
% b = 0;
A =[];
b = [];
Aeq = [];
beq = [];
lb = [0 0 0 0 0  20];
baseline = 1;
FOV = 50;
resH = 500;
ub = [100 100 100 100 100 (baseline/2)/tand(((FOV/2)/(resH/2))/2)];
x = fmincon(@simulate, gains, A,b,Aeq,beq,lb,ub,[], options);

%%
gains = x;
[ts, total_error, broke] = simulate_dev(gains,1);

total_error(total_error == 0) = [];
dt = 1e-1;
num_steps = length(total_error);
tfinal = num_steps*dt;
tspan = dt:dt:tfinal;

figure()
plot(tspan,total_error); hold on
xlabel('Time (sec)')
ylabel('Total Error')
yl = ylim;
plot([broke broke],[yl(1) yl(2)],'--k')