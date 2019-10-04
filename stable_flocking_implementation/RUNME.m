clear; matlabrc; clc; close all;
addpath(genpath('controllers'))
addpath(genpath('dynamics'))
addpath(genpath('tools'))
rng(1); %(random seed for repeatability)

%% Settings:
% Basic Simulation Settings:
write_to_video = true;
dt = 1;
tspan = 0:dt:300;
WIDTH = 150; %(half width of the field)

% Number of agents/obstacles to simulate:
num_agents = 6;
num_obstacles = 10;

% Virtaul leader (position and velocity):
vl = [-70 -50 1 1]; %(rx,ry,vx,vy)

% Distance settings:
d0 = 20; % Desired distance from virtual leader
d_react_obsr = 1.5; %Reaction distance (in terms of obstacle radius)

% Agent limitations:
max_v = 5; %(Max velocity allowed)
max_u = 1; %(Max acceleration allowed)

% Control gains:
k_ria = 20;  %(inter-agent position)
k_via = .1;  %(inter-agent velocities)
k_rvl = .25; %(virtual-leader position)
k_vvl = .5;  %(virtual-leader velocity)
k_obs = 5;   %(obstacle position)

%% Initialization:
% Memory allocation:
L = length(tspan);
u = zeros(2*num_agents,L);
r = zeros(2*num_agents,L);
v = zeros(2*num_agents,L);
vl_rv = zeros(4,L);

% Randomly place obstacles:
obs = [WIDTH*rand(num_obstacles,1) WIDTH*rand(num_obstacles,1), 5*rand(num_obstacles,1)+5];
d_react = d_react_obsr*obs(:,3); % react distance defined off of obstacle size

% Randomly place the agents:
r(:,1) = 20*randn(2*num_agents,1);
v(:,1) = 1*randn(2*num_agents,1);
vl_rv(:,1) = vl';


%% Run Simulation Loop:
for ii = 1:L-1
    % Propagate the dynamics:
    X_out = RK4(@equations_of_motion,dt,[r(:,ii);v(:,ii)],u(:,ii));
    r(:,ii+1) = X_out(1:2*num_agents);
    v(:,ii+1) = X_out(2*num_agents+1:4*num_agents);
    vl_rv(:,ii+1) = RK4(@equations_of_motion,dt,vl_rv(:,ii),[0;0]);
    
    % Calculate the control:
    u(:,ii+1) = controller(r(:,ii+1),v(:,ii+1),vl_rv(:,ii+1)',obs,d0,d_react,...
                           k_ria,k_via,k_rvl,k_vvl,k_obs);
    
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

%% Create flock animation:
if write_to_video
    
   vid = VideoWriter('flock_animation.avi','Uncompressed AVI'); 
   open(vid)
end

figure(1)
    % Initialize the animation:
    jj = 1;
    virtual_leader = plot(vl_rv(1,1),vl_rv(2,1),'sr','MarkerSize',10,'MarkerFaceColor','r'); hold on
    obstacles = gobjects(num_obstacles,1);
    for ii = 1:num_obstacles
        obstacles(ii) = circle(obs(ii,1),obs(ii,2),obs(ii,3));
    end
    agents = gobjects(num_agents,1);
    for ii = 1:2:2*num_agents
        agents(jj) = plot(r(ii,1),r(ii+1,1),'.','MarkerSize',20);
        jj = jj+1;
    end
    axis equal
    grid on
    legend([virtual_leader, agents(1),obstacles(1)],...
           'Virtual Leader','Agents','Obstacles',...
           'location','northwest')

    % Actuall show the animation:
    for ii = 1:L
        % Add track history:
        r_vec = reshape(r(:,ii)',2,[])';
        plot(r_vec(:,1),r_vec(:,2),'.','color',[.5 .5 .5],'MarkerSize',2); hold on
        for jj = 1:num_agents
            set(agents(jj),'XData',r_vec(jj,1),'YData',r_vec(jj,2));
        end
        set(virtual_leader,'XData',vl_rv(1,ii),'YData',vl_rv(2,ii));
        xlim([-WIDTH/2 1.5*WIDTH])
        ylim([-WIDTH/2 1.5*WIDTH])
        drawnow
        if write_to_video
            frame = getframe(gcf);
            writeVideo(vid,frame);
        else
            pause(.01)
        end
    end
    if write_to_video
        close(vid)
    end