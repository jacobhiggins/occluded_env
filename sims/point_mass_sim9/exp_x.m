% Find expected distance x that the 
function [exp_x,exp_frac] = exp_x(unc_probe)
    d = unc_probe.d;
    dist = d;
    p = 1;
    num = length(unc_probe.prob);
    exp_x = 0;
    for i = 1:num
        exp_x = exp_x + dist*p*(unc_probe.prob(i));
        dist = dist + d;
        p = p*(1-unc_probe.prob(i));
    end
    exp_x = exp_x + dist*p;
    exp_frac = exp_x/dist;
end