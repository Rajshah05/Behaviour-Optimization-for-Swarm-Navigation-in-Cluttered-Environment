function [settled,total_error] = settle_check(r,v,vl,d0)
    allowable_error = 0;
    
    settled = false;
    total_error = 0;
    num_agents = length(r)/2;
    
    % Reformatting:
    r_vec = reshape(r',2,[])';
    v_vec = reshape(v',2,[])';
    
    % Calculate error signals:
    [~,vl_d] = normr(r_vec - vl(1:2));
    vl_dx = vl_d - d0;
%     dia = num_agents*2*d0*sind(180/num_agents);
    for ii = 1:num_agents
        e_via = norm(sum(v_vec(ii,:) - v_vec([1:ii-1,ii+1:end],:),1));
        e_vvl = norm(sum(v_vec(ii,:) - vl(3:4),1));
        total_error = total_error + sqrt(e_via.^2 + sum(vl_dx).^2 + e_vvl.^2);
    end
    
    % Must all be past the obstacle (heading away from it):
    
    if total_error < allowable_error
        settled = true;
    end
end