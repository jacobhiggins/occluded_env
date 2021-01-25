function circ_pts = circle(x,y,r)
    theta = 0:0.05:2*pi;
    circ_pts.x = x + r*cos(theta);
    circ_pts.y = y + r*sin(theta);
end