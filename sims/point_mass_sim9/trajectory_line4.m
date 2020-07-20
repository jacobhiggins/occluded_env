% Keep pusuit a certain distance away from pm
% Want it to go certain velocity in y, position in x
function [xg,yg,vxg,vyg,v,vmax] = trajectory_line4(x,y,dt,v,xc,yc,xg_prev,yg_prev,hls,hws,wypts,M,sec)
    g_dist = 2.5; % Distance away that the pursuit stays
    if sec < 3
        wypt_i = sec + 1;
    else
        wypt_i = sec;
    end
    dvec = [0,1];
    if sec~=3
       dvec = wypts(sec+1,:) - wypts(sec,:);
       dvec = dvec/norm(dvec,2);
    end
    a = (dvec*([x;y] - wypts(sec,:)'))*dvec + g_dist*dvec;
%     [dx,dy] = c2u(g_dit*dvec(1),g_dist*dvev(2),0,0,M);
%     [a,b] = u2c(dx,dy,0,0,M);
    xg = wypts(sec,1)' + a(1);
    yg = wypts(sec,2)' + a(2);
    
    
    dx2 = xc - x;
    dy2 = yc - y; % How far I am from corner
    [dx2,dy2] = c2u(dx2,dy2,0,0,M);
    ym2 = 100;
    if sec==1 || sec==2
        ym2 = hws(sec+1);
    end
    [v,vmax] = safety_vel(-dx2,-dy2,0,0,0,ym2,0,0);
    [c,d] = u2c(0,v,0,0,M);
    vxg = c;
    vyg = d;
end