function val = cost(time2clear,maxKU,vx_zero,stuck)
    val = -1000000*time2clear + ...
        0*maxKU + ...
        -10*max(vx_zero-1,0) + ...
        -500*stuck;
%     val = max(val,-500);
end