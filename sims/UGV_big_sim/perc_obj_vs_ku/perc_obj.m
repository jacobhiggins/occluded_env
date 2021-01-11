function po = perc_obj(p)
    x = p.x;
    y = p.y;
    po = (atan(y./x)./y).^2;
end