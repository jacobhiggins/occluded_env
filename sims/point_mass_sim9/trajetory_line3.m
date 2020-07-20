% Keep pusuit a certain distance away from pm
% Want it to go certain velocity in y, position in x
function [xg,yg,vxg,vyg] = trajetory_line3(x,y,dt,xc,yc,xg_prev,yg_prev,wypts,M,sec)
    g_dist = 5; % Distance away that the pursuit stays
    if sec < 3
        wypt_i = sec + 1;
    else
        wypt_i = sec;
    end
    dvec = [x;y] - [xg_prev,yg_prev];
    [dx,dy] = c2u(dev(1),dvec(2),0,0,M);
    dy = min(dy,g_dist);
    a = u2c(dx,dy,0,0,M);
    xg = x + a(1);
    yg = y + a(2);
    
    
    dx2 = xc - x;
    dy2 = yc - y; % How far I am from corner
    ym2 = 100;
    if sec==1 || sec==2
        ym2 = hws(sec+1)/2;
    end
    [v,vmax] = safety_vel(-dx2,-dy2,0,0,0,ym2,0);
    b = u2c(0,v,0,0,M);
    vxg = b(1);
    vyg = b(2);
end