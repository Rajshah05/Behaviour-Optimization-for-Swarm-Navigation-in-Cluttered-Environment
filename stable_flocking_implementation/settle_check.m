function [settled] = settle_check(r,v,vl,d0)
    allowable_error = 0;
    
    settled = false;
    total_error = 0;
    num_agents = length(r)/2;
    
    % Reformatting:
    r_vec = reshape(r',2,[])';
    v_vec = reshape(v',2,[])';
    
    % Calculate error signals:
    [vl_dirs,vl_d] = normr(r_vec - vl(1:2));
    vl_dx = vl_d - d0;
    for ii = 1:num_agents
        [ia_dirs,ia_d] = normr(r_vec(ii,:) - r_vec([1:ii-1,ii+1:end],:));
        e_ria = sum(ia_dirs./ia_d);
        e_via = sum(v_vec(ii,:) - v_vec([1:ii-1,ii+1:end],:),1);
        e_rvl = vl_dirs(ii,:)*vl_dx(ii);
        e_vvl = sum(v_vec(ii,:) - vl(3:4),1);
        total_error = total_error + sum(e_ria.^2 + e_via.^2 + e_rvl.^2 + e_vvl.^2);
    end
    
    % Must all be past the obstacle (heading away from it):
    
    
%     disp(total_error)
    if total_error < allowable_error
        settled = true;
    end
end