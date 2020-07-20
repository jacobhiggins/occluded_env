% All corners are known a priori
% Return current corner and correct M matrix
% When robot+currentcorner angle equals robot+nextcorner angle, advance
% current corner
function [xc,yc,M,flip,c] = setCorner2(x,y,corners,Ms)
    persistent curr_c;
    persistent next_c;
    if isempty(curr_c)
       curr_c = 1;
       next_c = 2;
    end
    curr_c = min(curr_c,size(corners,1));
    next_c = min(next_c,size(corners,1));
    xc1 = corners(curr_c,1);
    yc1 = corners(curr_c,2);
    xc2 = corners(next_c,1);
    yc2 = corners(next_c,2);
    avoid = corners(curr_c,3);
    phi1 = atan2(yc1-y,xc1-x);
    phi2 = atan2(yc2-y,xc2-x);
    if sign(phi2 - phi1) == avoid
       curr_c = curr_c + 1;
       next_c = next_c + 1;
    end
    xc = corners(curr_c,1);
    yc = corners(curr_c,2);
    M = Ms{curr_c};
    flip = corners(curr_c,3);
    c = curr_c;
end