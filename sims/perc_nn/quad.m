function cost = quad(v)
    x = v(1,:);
    y = v(2,:);
    cost = x.^2 + y.^2;
end