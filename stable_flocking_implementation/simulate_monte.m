function [ts_avg,ts,total_error,broke,ii,vl_rv,r,u] = simulate_monte(gains)
    fprintf('gains: %f %f %f %f %f\n', gains)
    
    %% Run simulation:
    number_of_runs = 100;
    ts = zeros(number_of_runs,1);
    for run = 1:number_of_runs
        init_monte
        total_error = zeros(L-1,1);
        formation_start = false;
        settle_count = 0;
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

            % Apply limit to max velocity:
    %         v_vec = reshape(v(:,ii+1)',2,[])';
    %         v_vec2 = v_vec + u_vec*dt;
    %         [~,v2_norms] = normc(v_vec2);
    %         if any(v2_norms > max_v)
    %             u_vec = u_vec -v(v2_norms < max_v)/dt;
    %         end
            u(:,ii+1) = reshape(u_vec',[],1);

            % Check if any agents got too close to obstacle:
            obstacle_violation = obstacle_violation_check(r(:,ii),obs,d_min);
            if obstacle_violation
                ts(run) = inf;
                disp('Run Failed (Obstacle Hit)')
                break 
            end

            % Check if settled:
            obs2vl = vl_rv(1:2,ii)' - obs(1:2);
            theta = acos(dot(obs2vl,vl_rv(3:4,ii)')/(norm(vl_rv(3:4,ii)')*norm(obs2vl)));
            [settled,total_error(ii)] = settle_check(v(:,ii),vl_rv(:,ii)');
            if theta < pi/2 % Passed the obstacle
                if settled
                    break
                end
            end

            if ~formation_start
                if settled
                    if settle_count > 30
                        formation_start = true;
                    end
                    settle_count = settle_count+1;
                else
                    settle_count = 0;

                end
            end

            if formation_start && ~settled
                if ts(run) == 0
                    broke = ii*dt;
                end
                ts(run) = ts(run)+dt;
            end
        end
        fprintf('run: %d, settled: %f (sec)\n',run,ts(run))
    end
    ts_avg = mean(ts);
    fprintf('mean settling time: %f\n\n',ts_avg)
end