function [c, c_wypt, c_perc] = cost(x,y,xw,yw)
    c_wypt = (x-xw).^2 + (y-yw).^2;
    c_perc = 100*atan2(y,x)./y;
    c = c_wypt + c_perc;
end