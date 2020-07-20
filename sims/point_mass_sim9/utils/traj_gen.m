% Generate trajectory, minimum jerk
function [ xgs, ygs, vxgs, vygs, axgs, aygs ] = traj_gen(way_pts,v,dt)
    v = 5;
    start = way_pts(1:end-1,:);
    stop = way_pts(2:end,:);
    dis = sqrt(sum((stop - start).^2, 2));
    num = size(start, 1);
    coeff_x = zeros(6,num);
    coeff_y = coeff_x;
    scale = 1;
    % Find coefficients for trajectory generation
    for i = 1:num
        x0 = start(i,1);
        xT = stop(i,1);
        y0 = start(i,2);
        yT = stop(i,2);
        d = dis(i);
        T = scale*d/v;
        A = [0,      0,      0,     0,   0   1;
             T^5,    T^4,    T^3,   T^2, T,  1;
             0,      0,      0,     0,   1,  0;
             5*T^4,  4*T^3,  3*T^2, 2*T, 1,  0;
             0,      0,      0,     2,   0,  0;
             20*T^3, 12*T^2, 6*T,   2,   0,  0];
        coeff_x(:,i) = A \ [x0 xT 0 0 0 0]';
        coeff_y(:,i) = A \ [y0 yT 0 0 0 0]';
    end
    % With these coefficients, find trajectories
    
    xgs = [];
    ygs = [];
    vxgs = [];
    vygs = [];
    axgs = [];
    aygs = [];
    
    for i = 1:num
        d = dis(i);
        T = scale*d/v;
        for t = 0:dt:T
           s = [t^5, t^4, t^3, t^2, t, 1];
           sd = [5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0];
           sdd = [20*t^3, 12*t^2, 6*t, 2, 0, 0];
           xg = s * coeff_x(:,i);
           yg = s * coeff_y(:,i);
           vxg = sd * coeff_x(:,i);
           vyg = sd * coeff_y(:,i);
           axg = sdd * coeff_x(:,i);
           ayg = sdd * coeff_y(:,i);
           xgs = [xgs xg];
           ygs = [ygs yg];
           vxgs = [vxgs vxg];
           vygs = [vygs vyg];
           axgs = [axgs axg];
           aygs = [aygs ayg];
        end
    end
end