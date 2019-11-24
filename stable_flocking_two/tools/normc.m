function [M,n] = normc(M)
    n = sqrt(sum(M.^2,1));
    M = bsxfun(@rdivide,M,n);
end