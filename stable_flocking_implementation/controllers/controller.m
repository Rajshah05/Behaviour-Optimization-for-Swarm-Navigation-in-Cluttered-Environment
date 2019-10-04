function [u] = controller(r,v,vl,obs,d0,d_react,k_ria,k_via,k_rvl,k_vvl,k_obs)
    % Actual controller implementation:
    num_agents = length(r)/2;
    
    % Reshape the inputs into a more convenient vector form:
    r_vec = reshape(r',2,[])';
    v_vec = reshape(v',2,[])';
    
    % Control law:
    u = zeros(num_agents,2);
    
    % Control gains:
    [vl_dirs,vl_d] = normr(r_vec - vl(1:2));
    vl_dx = vl_d - d0;
    for ii = 1:num_agents
        % Obstacles:
        e_robs = [0,0];
        for jj = 1:size(obs,1)
            [obs_dirs,obs_d] = normr(r_vec - obs(jj,1:2));
            e_robs = e_robs + obs_dirs(ii,:)*(d_react(jj)/obs_d(ii))^2;
        end
        % Formulate error signals:
        [ia_dirs,ia_d] = normr(r_vec(ii,:) - r_vec([1:ii-1,ii+1:end],:));
        e_ria = sum(ia_dirs./ia_d);
        e_via = sum(v_vec(ii,:) - v_vec([1:ii-1,ii+1:end],:),1);
        e_rvl = vl_dirs(ii,:)*vl_dx(ii);
        e_vvl = sum(v_vec(ii,:) - vl(3:4),1);
        
        
        % Apply gains:
        u(ii,:) = k_ria*e_ria -k_via*e_via -k_rvl*e_rvl -k_vvl*e_vvl + k_obs*e_robs;
                  
    end
    
    % Reformulate:
    u = reshape(u',[],1);
end