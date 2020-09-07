function [x,y] = newtonsMethod(x0,y0,c,fx,fy,J)
    % Given initial guess x0, y0
    % Find zero of multivariate funciton f = (fx, fy)
    % J = jacobian of f
    n = 1000000;
    epsilon = 0.001;
    
    old = [x0;y0];
    new = old + 5*epsilon*ones(2,1);
    i = 0;
    while norm(old-new,2)>epsilon && i < n
       old = new;
       new = old - J(old(1),old(2),c)\[fx(old(1),old(2),c,x0);fy(old(1),old(2),c,y0)];
       i = i + 1;
    end
    
    if i==n
       disp("Warning: 1000 steps reached, answer given may be inaccurate"); 
    end
    
    x = new(1);
    y = new(2);
end