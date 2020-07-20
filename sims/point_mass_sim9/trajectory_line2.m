% Include different directions that the pursuit can move (from waypoint to
% waypoint
function [xg,yg] = trajectory_line2(x,y,xc,yc,xg_prev,yg_prev,waypoints,dt,hws)
    persistent wypt_i; % Waypoint index, which waypoint we are going towards
    if isempty(wypt_i)
        wypt_i = 2;
    elseif wypt_i == length(waypoints)
       wypt_i = length(waypoints); 
    else
       wypt_b = waypoints(wypt_i,:);
       if norm([yg_prev-wypt_b(2),xg_prev-wypt_b(1)],2) < 1
           wypt_i = wypt_i + 1; % If pursuit is close enough to next waypoint, switch future we're headed towards
       end
    end
    wypt_a = waypoints(wypt_i-1,:); % Starting wypt
    wypt_b = waypoints(wypt_i,:);  % End wypt
    d_vec = wypt_b - wypt_a; % Direction vector
    d_vec = d_vec/norm(d_vec,2); % Normalize vector
    
    dx = xc - xg_prev;
    dy = yc - yg_prev; % How far I am from corner, determines velocity of pursuit
    D = sqrt((x-xc)^2 + (y-yc)^2); % Distance between point mass and corner
    phi_norm = 1 - abs(atan((yc-y)/(xc-x)))/(pi/2);
    phi = abs(atan((yc-y)/(xc-x)))/(pi/2);
    v = set_traj_vel2(D,hws(1),phi_norm,phi);
    if (dx^2 + dy^2) < 0.1
       v = 10; 
    end
    
    g_pos = [xg_prev,yg_prev] + v*d_vec*dt;
    xg = g_pos(1);
    yg = g_pos(2);
end