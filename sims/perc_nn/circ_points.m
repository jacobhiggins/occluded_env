function circ_ps = circ_points(x,y,rad)
    thetas = 0.0:0.01:2*pi;
    circ_ps.x = zeros(1,length(thetas));
    circ_ps.y = zeros(1,length(thetas));
    for i = 1:length(thetas)
       theta = thetas(i);
       circ_ps.x(i) = x + rad*cos(theta);
       circ_ps.y(i) = y + rad*sin(theta);
    end
end