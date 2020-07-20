function [v,vmax] = safety_vel2(del_x)
    a_max = 1;
    vmax = sqrt(2*delta_x*a_max);
    v = min(sqrt(2*a_max*delta_x),vmax);
end