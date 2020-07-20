% Return xs and ys of uncertainty probe

function unc_probe = get_probe(x,y,xinsct,yintsct,theta2,dmax,patches)
    num = 100; % num of points
    d = dmax/num;
    vec = [xinsct-x;yintsct-y];
    dshort = norm(vec,2);
    vec = vec/dshort;
    unc_probe.x = [];
    unc_probe.y = [];
    unc_probe.prob = [];
    d1 = d;
    d2 = d;
    for i = 1:num
       if d1 < dshort % From robot to waypoint
           xp = x + vec(1)*d1;
           yp = y + vec(2)*d1;
           d1 = d1 + d;
       else % From waypoint towards next waypoint
           xp = xinsct + cos(theta2)*d2;
           yp = yintsct + sin(theta2)*d2;
           d2=d2+d;
       end
       p = get_patch_prob(xp,patches);
       unc_probe.x = [unc_probe.x;xp];
       unc_probe.y = [unc_probe.y;yp];
       unc_probe.prob = [unc_probe.prob;p];
    end
    unc_probe.d = d;
    unc_probe.num = num;
    unc_probe.dmax = dmax;
end