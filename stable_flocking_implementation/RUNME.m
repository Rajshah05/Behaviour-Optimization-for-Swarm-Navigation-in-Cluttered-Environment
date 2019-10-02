clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))

% Time settings:
dt = 1;
duration = 60;
tspan = 0:dt:duration;

% Agent definitions:
num_agents = 5;

% Memory allocation:
L = length(tspan);
u = zeros(2*num_agents,L);
r = zeros(2*num_agents,L);
v = zeros(2*num_agents,L);

% Initial conditions
r(:,1) = 20*randn(2*num_agents,1);
v(:,1) = 1*randn(2*num_agents,1);

max_v = 5;
max_u = 1e-1;

for ii = 1:L-1
    % Propagate the dynamics:
    X_out = RK4(@equations_of_motion,dt,[r(:,ii);v(:,ii)],u(:,ii));
    r(:,ii+1) = X_out(1:2*num_agents);
    v(:,ii+1) = X_out(2*num_agents+1:end);
    
    % Calculate the control:
    u(:,ii+1) = controller(r(:,ii+1),v(:,ii+1));
    
    % Apply limitations on control input:
    u_vec = reshape(u(:,ii+1)',2,[])';
    [u_norm,u_norms] = normr(u_vec);
    u_vec(u_norms > max_u,:) = u_norm(u_norms > max_u,:)*max_u;
    
    % Apply limit to max velocity:
    v_vec = reshape(v(:,ii+1)',2,[])';
    v_vec2 = v_vec + u_vec*dt;
    [v2_norm,v2_norms] = normc(v_vec2);
    u_vec(v2_norms > max_v) = 0;
    
    u(:,ii+1) = reshape(u_vec',[],1);
end

% Plot the results:
figure()
    subplot(2,3,1)
        for ii = 1:2:2*num_agents
            plot(tspan,r(ii,:)); hold on
        end
        grid on
        title('R_x')
    subplot(2,3,4)
        for ii = 1:2:2*num_agents
            plot(tspan,r(ii+1,:)); hold on
        end
        grid on
        title('R_y')
        
    subplot(2,3,2)
        for ii = 1:2:2*num_agents
            plot(tspan,v(ii,:)); hold on
        end
        grid on
        title('V_x')
    subplot(2,3,5)
        for ii = 1:2:2*num_agents
            plot(tspan,v(ii+1,:)); hold on
        end
        grid on
        title('V_y')
        
    subplot(2,3,3)
        for ii = 1:2:2*num_agents
            plot(tspan,u(ii,:)); hold on
        end
        grid on
        title('U_x')
    subplot(2,3,6)
        for ii = 1:2:2*num_agents
            plot(tspan,u(ii+1,:)); hold on
        end
        grid on
        title('U_y')