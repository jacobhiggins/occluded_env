function circ_shape = get_circ(x,y,r)
    theta = 0:0.1:2*pi;
    circ_shape.x = r*cos(theta) + x;
    circ_shape.y = r*sin(theta) + y;
end