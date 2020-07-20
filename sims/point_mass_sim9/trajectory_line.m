% Use with MPC5
function [xg,yg,vxg,vyg,v,v_set] = trajectory_line(x,y,dt,v,xc,yc,xg_prev,yg_prev,hls,hws,M,sec)
    dx = xc - xg_prev;
    dy = yc - yg_prev; % How far I am from corner, determines velocity of pursuit
    D = sqrt((x-xc)^2 + (y-yc)^2); % Distance between point mass and
    phi = abs(atan((yc-y)/(xc-x)));
    phi_norm = 1 - abs(atan((yc-y)/(xc-x)))/(pi/2);
    dx2 = xg_prev - x;
    dy2 = yg_prev - y; % How far I am from pursuit, determines update
    dx3 = xc - x;
    dy3 = yc - y; % How far I am from corner
    [dx,dy] = c2u(dx,dy,0,0,M); % Rotate based on section
    [dx2,dy2] = c2u(dx2,dy2,0,0,M); % Rotate based on section
    [dx3,dy3] = c2u(dx3,dy3,0,0,M);
    v_set = 10;
    
    stop_dist = 3*(10^2/(2*10));
    if dy < stop_dist
       x = 3; 
    else
       x = 1; 
    end
%     v = 10;
%     v_set = set_traj_vel(x);
%     v = set_traj_vel2(D,hws(1),phi_norm,phi);

    ym2 = 100;
    if sec==1 || sec==2
        ym2 = hws(sec+1)/2;
    end
    [v,vmax] = safety_vel(-dx3,-dy3,0,0,0,ym2,0);
    
    
    
%     v_set = 10*(D/(hws(1)/2 + D));
%     disp(v_set);
    if (dx^2 + dy^2) < 0.1
       v = 10; 
    end
    amax = 10;
%     if abs(v - v_set) > 0.1
%        v = v - (v - v_set)/(abs(v)/amax)*dt; 
%     end
    
%     vmax = 10;
%     vslow = 1;
%     stop_dist = 5*(vmax^2/(2*amax));
%     if dy < stop_dist && xg_prev < hl2 - hw3/2 && v > vslow
%        v = v - (v - vslow)/(vmax/amax)*dt; 
%     else
%        v = vmax; 
%     end
    if abs(dy2) < 10
        if yg_prev < hls(1)-hws(2)/2 || xg_prev > hls(2)-hws(3)/2
            xg = xg_prev;
            yg = yg_prev + v*dt;
            vyg = v;
            vxg = 0;
        else
%             xg = xg_prev + v*dt;
            xg = xg_prev + v*dt;
            yg = yg_prev;
            vyg = 0;
            vxg = v;
        end
    else
       xg = xg_prev;
       yg = yg_prev;
       vyg = 0;
       vxg = 0;
    end
end