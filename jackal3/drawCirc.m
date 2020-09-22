function points = drawCirc(r,x,y)
    points.x = r*cos(0:0.1:2*pi) + x;
    points.y = r*sin(0:0.1:2*pi) + y;
end