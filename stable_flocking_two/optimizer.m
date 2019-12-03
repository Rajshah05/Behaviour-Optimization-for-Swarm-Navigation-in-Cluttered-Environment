clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Initial Control gains:
k_ria = 20;  %(inter-agent position)
k_via = 30;  %(inter-agent velocities)
k_rvl = 50; %(virtual-leader position)
k_vvl = 20;  %(virtual-leader velocity)
k_obs = 30;   %(obstacle position)
obs_dist = 50;
gains = [k_ria,k_via,k_rvl,k_vvl,k_obs,obs_dist]';

% Optimize:
options = optimoptions('fmincon','FiniteDifferenceStepSize',[1e-2 1e-1 1e-1 1e-1 1e-2 1e-2]);
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

ts_history = [];
min_dist = [];
save OUT ts_history min_dist
[x,~,~,output] = fmincon(@simulate, gains, A,b,Aeq,beq,lb,ub,[], options);

load OUT
plot(min_dist,'*r')

% for 50:
% 20.721258 0.003045 71.851697 18.776085 29.772703 49.835009
% 19.818573 0.002823 67.835043 19.276659 29.702583 39.892292

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