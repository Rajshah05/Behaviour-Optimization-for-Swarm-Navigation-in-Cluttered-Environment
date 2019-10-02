function [u] = controller(r,v)
    % Actual controller implementation:
    num_agents = length(r)/2;
    
    % Reshape the inputs into a more convenient vector form:
    r_vec = reshape(r',2,[])';
    v_vec = reshape(v',2,[])';
    
    % Potential function:
    
    % Control law:
    u = zeros(num_agents,2);
    kp_v = .1;
    kp_grad = .1;
    for ii = 1:num_agents
       u(ii,:) = -kp_v*sum(v_vec(ii,:) - v_vec([1:ii-1,ii+1:end],:),1) + ...
                 -kp_grad*sum(0);
    end
    
    % Reformulate:
    u = reshape(u',[],1);
end