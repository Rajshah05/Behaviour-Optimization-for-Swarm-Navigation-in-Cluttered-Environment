function [dX] = equations_of_motion(~,X,u)
    L = length(X);
    dX = [X(((L/2)+1):end); u];
end