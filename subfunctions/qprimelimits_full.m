function [lb,ub] = qprimelimits_full(qlimit,qprev,N,qpmax,qpmin)

% Compute limits due to joint stops
lb_js = N*(qlimit(:,1) - qprev);
ub_js = N*(qlimit(:,2) - qprev);

% Compare and find most restrictive bound
lb = zeros(9,1); ub = zeros(9,1); ub(8) = 1; ub(9) = 1;
for k = 1:7
    if lb_js(k) > qpmin(k)
        lb(k) = lb_js(k);
    else
        lb(k) = qpmin(k);
    end
    
    if ub_js(k) < qpmax(k)
        ub(k) = ub_js(k);
    else
        ub(k) = qpmax(k);
    end
end

end