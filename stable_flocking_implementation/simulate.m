function [ts,total_error] = simulate(gains)
    %% Setup:
    disp(gains)
    rng(1); %(random seed for repeatability)
    dt = 1e-3;
    tspan = 0:dt:3000;

    % Number of agents/obstacles to simulate:
    num_agents = 4;
    num_obstacles = 1;
    
    % Distance settings:
    d0 = 20; % Desired distance from virtual leader
    d_react_obsr = 1.5; %Reaction distance (in terms of obstacle radius)
    d_min_obsr = 1; %Minimum allowable distance to obstalce (in terms of obstacle radius)
    % IF THE AGENT GOES BELOW THE MINIMUM DISTANCE, THE RUN FAILS.
    
    % Agent limitations:
    max_v = 1; %(Max velocity allowed)
    max_u = 1; %(Max acceleration allowed)
    
    % Virtaul leader (position and velocity):
    vl = [-70 -70 1 1]; %(rx,ry,vx,vy)
    
    %% Initialization:
    % Memory allocation:
    L = length(tspan);
    u = zeros(2*num_agents,L);
    r = zeros(2*num_agents,L);
    v = zeros(2*num_agents,L);
    vl_rv = zeros(4,L);

    % Randomly place obstacles:
    obs_radii = 20*ones(num_obstacles,1); % 5*rand(num_obstacles,1)+5;
    obs = [20 20, obs_radii];
    d_react = d_react_obsr*obs(:,3); % react distance defined off of obstacle size
    d_min = d_min_obsr*obs(:,3);

    % Randomly place the agents:
    r(:,1) = repmat(reshape(vl(1:2),[],1),num_agents,1) + randn(2*num_agents,1);
    v(:,1) = 1*randn(2*num_agents,1);
    vl_rv(:,1) = vl';
    
    %% Run simulation:
    ts = 0;
    total_error = zeros(L-1,1);
    for ii = 1:L-1
        % Propagate the dynamics:
        X_out = RK4(@equations_of_motion,dt,[r(:,ii);v(:,ii)],u(:,ii));
        r(:,ii+1) = X_out(1:2*num_agents);
        v(:,ii+1) = X_out(2*num_agents+1:4*num_agents);
        vl_rv(:,ii+1) = RK4(@equations_of_motion,dt,vl_rv(:,ii),[0;0]);

        % Calculate the control:
        u(:,ii+1) = controller(r(:,ii+1),v(:,ii+1),vl_rv(:,ii+1)',obs,d0,d_react,gains);

        % Apply limitations on control input:
        u_vec = reshape(u(:,ii+1)',2,[])';
        [u_norm,u_norms] = normr(u_vec);
        u_vec(u_norms > max_u,:) = u_norm(u_norms > max_u,:)*max_u;

        % Apply limit to max velocity: TODO: FIX THIS
%         v_vec = reshape(v(:,ii+1)',2,[])';
%         v_vec2 = v_vec + u_vec*dt;
%         [~,v2_norms] = normc(v_vec2);
%         u_vec(v2_norms > max_v) = 0;
        u(:,ii+1) = reshape(u_vec',[],1);
        
        % Check if any agents got too close to obstacle:
        obstacle_violation = obstacle_violation_check(r(:,ii),obs,d_min);
        if obstacle_violation
            ts = 1e5;
            disp('Run Failed (Obstacle Hit)')
            break 
        end
        
        % Check if settled:
        [settled,total_error(ii)] = settle_check(v(:,ii),vl_rv(:,ii)');
        if settled
            fprintf('settled: %f (sec)\n',ii*dt)
            break
        end
        
        % Increase settling time:
        ts = ts+dt;
    end
end