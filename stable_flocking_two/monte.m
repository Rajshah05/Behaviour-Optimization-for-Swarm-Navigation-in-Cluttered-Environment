clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Optimize:
options = optimoptions('fmincon','FiniteDifferenceStepSize',1e-1);
A =[];
b = [];
Aeq = [];
beq = [];
lb = [0 0 0 0 0  20];
baseline = 1;
FOV = 50;
resH = 500;
ub = [100 100 100 100 100 (baseline/2)/tand(((FOV/2)/(resH/2))/2)];

for ii = 1:10
    ts_history = [];
    save OUT ts_history
    k_ria = 20;  %(inter-agent position)
    k_via = 3;  %(inter-agent velocities)
    k_rvl = .5; %(virtual-leader position)
    k_vvl = 2;  %(virtual-leader velocity)
    k_obs = 30;   %(obstacle position)
    obs_dist = 40;
    k_ria2 = 10;  %(inter-agent position)
    k_via2 = 2;  %(inter-agent velocities)
    k_rvl2 = 2; %(virtual-leader position)
    k_vvl2 = 2;  %(virtual-leader velocity)
    k_obs2 = 2;   %(obstacle position)
    obs_dist2 = 10;
    gains = [k_ria,k_via,k_rvl,k_vvl,k_obs,obs_dist]' + [k_ria2,k_via2,k_rvl2,k_vvl2,k_obs2,obs_dist2]'.*rand(6,1);
    x = fmincon(@simulate, gains, A,b,Aeq,beq,lb,ub,[], options);

    load OUT
    plot(ts_history,'*'); hold on
    drawnow
end