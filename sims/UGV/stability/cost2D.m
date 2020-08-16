function [cs,cs_wypt,cs_perc] = cost2D(x,y,xw,perc_weight)
    cs_wypt = (x - xw).^2;
    cs_perc = perc_weight*(atan(y./x)./y).^2;
    cs = cs_wypt + cs_perc;
end