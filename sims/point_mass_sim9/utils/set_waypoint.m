% Given pos of robot (x,y)
% pos of corner (xc,yc)
% pos + orientation of waypoint base (xm2,ym2,theta2)
% Calculate position of waypoint to move towards
function waypoint = set_waypoint(x,y,xc,yc,xm2,ym2,theta2,hl)
    waypoint.x = xm2;
    waypoint.y = ym2;
    r2 = 0;
    theta = atan2(yc-y,xc-x);
    if theta ~= theta2
        A = [cos(theta),-cos(theta2);...
            sin(theta),-sin(theta2)];
        b = [xm2-x;ym2-y];
        vec = A\b;
        r2 = vec(2);
        if r2 < 0 || r2 > hl
            r2 = hl;
        end
    end
    waypoint.x = xm2 + r2*cos(theta2);
    waypoint.y = ym2 + r2*sin(theta2);
end