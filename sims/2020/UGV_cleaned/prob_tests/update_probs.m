function grid = update_probs(x,y,r,grid)
    global wall;
    I = 11;
    if wall && y < 4.5
        I = 6;
    end
    prob_z1_m1 = 0.9;
    prob_z0_m1 = 1 - prob_z1_m1;
    prob_z0_m0 = 0.9;
    prob_z1_m0 = 1 - prob_z0_m0;
    probs = grid.probs;
    p_appear = grid.p_appear;
    for i = 1:grid.n
       dist = norm([x-grid.xs(i),y-grid.ys(i)],2);
       if dist < r && i<I
           probs(i) = prob_z0_m0*probs(i)/(prob_z0_m0*probs(i) + prob_z1_m0*(1-probs(i)));
       elseif i==grid.n
           probs(i) = 0.5;
       else
           probs(i) = 0.1*probs(i+1) + 0.9*probs(i);
       end
    end
    for t = 1:uint16(abs(5-y)/(0.1))
       for i = 1:grid.n-1
          probs(i) = 0.1*probs(i+1) + 0.9*probs(i);
%             probs(i) = probs(i+1);
       end
    end
    grid.probs = probs;
end