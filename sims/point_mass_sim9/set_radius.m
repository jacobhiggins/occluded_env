% Set radius of waypoint circle
function r = set_radius(x, y, xc, yc, xm2, ym2, theta2, prev_rad, rad_max, c, dist_frac)
    r = rad_max;
    theta = atan2(yc-y,xc-x);
    if theta ~= theta2
        A = [cos(theta),-cos(theta2);...
            sin(theta),-sin(theta2)];
        b = [xm2-x;ym2-y];
        vec = A\b;
        r = vec(1);
%         disp(dist_frac);
        if (r > prev_rad && r < rad_max) || r < 0
            if r > 0
                r = min(prev_rad + rad_max/50,r);
            else
                r = prev_rad + rad_max/50;
            end
        end
        r = min(dist_frac*rad_max,r);
        if r > rad_max
            r = min(prev_rad + rad_max/50,rad_max);
        end
%         if r < 0 || r > rad_max
%            r = rad_max; 
%         end
        
    end
    
end