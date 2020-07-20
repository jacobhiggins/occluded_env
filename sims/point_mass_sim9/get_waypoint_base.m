% Based on hallway and obstacle geometry, get waypoint bases for the entire
% map
function way_pt_base = get_waypoint_base(x,y,hls,hws,obss)
    way_pt_base = [x,y];
    way_pt_base = [way_pt_base;x,hls(1)-hws(2)/2];
    M2 = [0,1;1,0];
    for i = 1:length(obss)
       obs = obss{i};
       avoid1 = obs.avoid;
       x1 = obs.avoid*obs.w/2; % x coordinate of lower left
       y1 = -obs.h/2; % y coordinate of lower left
       x2 = x1; % x coordinate of upper left
       y2 = obs.h/2; % y coordinate of upper left
       [x1,y1] = u2c(x1,y1,obs.x,obs.y,M2);
       [x2,y2] = u2c(x2,y2,obs.x,obs.y,M2);
       y1 = (y1+(hls(1) + hws(2)*(avoid1-1)/2))/2; % Between object and wall of 2nd hallway
       y2 = (y2+(hls(1) + hws(2)*(avoid1-1)/2))/2;
       way_pt_base = [way_pt_base;x1,y1;x2,y2];
    end
    way_pt_base = [way_pt_base;hls(2)-hws(3)/2,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(3)+ hls(1)];
end