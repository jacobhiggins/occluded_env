function [xc,yc,flip] = setCorner(xm2,ym2,xr,yr,sec,hws,hls,obss,Ms)
    flip = 1;
    secs = [];
    for i = 1:length(obss)
        o = obss{i};
        secs = [secs o.get_sec()];
    end
    if sec==1
       xc = hws(1);
       yc = hls(1)-hws(2); % ******CHANGE BACK
    elseif sec==2
       % For now, assume all obstacles are in section 2
       is = find(secs==2);
       xc = hls(2) - hws(3);
       yc = hls(1);
       for i = is
          obs = obss{i};
          x1 = obs.avoid*obs.w/2; % x coordinate of lower left
          y1 = -obs.avoid*obs.h/2; % y coordinate of lower left
          [x1,y1] = u2c(x1,y1,obs.x,obs.y,Ms{sec});
          if xr < x1 && x1 < xc
              xc = x1;
              yc = y1;
              flip = -1*obs.avoid;
          end
       end
       
    else
       xc = xm2;
       yc = ym2;
    end
end