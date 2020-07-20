% Only advance current corner when past current corner
% 
function [xc,yc,M,flip,c] = setCorner3(x,y,xc,yc,corners,Ms,i)
    persistent curr_cs;
    if isempty(curr_cs)
       curr_cs = ones(2,1);
    end
    M = Ms{curr_cs(i)};
    [delx,dely] = c2u(x,y,xc,yc,M);
    if dely > 0
       curr_cs(i) = min(curr_cs(i) + 1,size(corners,1)); 
    end
    xc = corners(curr_cs(i),1);
    yc = corners(curr_cs(i),2);
    M = Ms{curr_cs(i)};
    flip = corners(curr_cs(i),3);
    c = curr_cs(i);
end