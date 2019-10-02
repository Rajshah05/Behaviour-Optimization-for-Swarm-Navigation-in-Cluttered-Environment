function [M,n] = normr(M)
    n = sqrt(sum(M.^2,2));
    M = bsxfun(@rdivide,M,n);
end