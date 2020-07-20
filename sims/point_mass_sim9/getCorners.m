% Get all the corners of a map
% Also get matrices associated with each corner
% Also get walls
function [corners,Ms,walls] = getCorners(hls,hws,obss)
    corners = [hws(1),hls(1)-hws(2),1]; % xc, yc, avoid (+1 clockwise, -1 counterclockwise)
    M2 = [0,1;-1,0];
    M3 = -1*eye(2);
    M4 = [0,1;1,0];
    Ms = {eye(2)};
    walls = [-1*hws(1)+1];
    for i = 1:length(obss)
       obs = obss{i};
       avoid1 = obs.avoid;
       x1 = obs.avoid*obs.w/2; % x coordinate of lower left
       y1 = -obs.h/2; % y coordinate of lower left
       x2 = x1; % x coordinate of upper left
       y2 = obs.h/2; % y coordinate of upper left
       [x1,y1] = u2c(x1,y1,obs.x,obs.y,M4);
       [x2,y2] = u2c(x2,y2,obs.x,obs.y,M4);
       corners = [corners;x1,y1,avoid1;x2,y2,avoid1];
       Ms = cat(2,Ms,{[1,0;0,avoid1]});
       Ms = cat(2,Ms,{[0,-1*avoid1;1,0]});
       walls = [walls -30 -1*(hws(2)-obs.w)];
%        Ms = cat(2,Ms,{eye(2)});
%        Ms = cat(2,Ms,{[0,1;1,0]});
    end
    % Corner at end of 2nd hallway
    walls = [walls -30];
    corners = [corners;hls(2)-hws(3),hls(1),-1];
    Ms = cat(2,Ms,[0,1;1,0]);
    
    % Corner at end of last hallway
    walls = [walls -30];
    corners = [corners;hls(2)-hws(3)/2,hls(1)+hls(3)-hws(2),1];
    Ms = cat(2,Ms,eye(2));
end