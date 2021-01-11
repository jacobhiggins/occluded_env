% Go from cartesian to u, v coordinates
function [u,v] = c2u(x,y,xc,yc,M)
    vec = [x;y] - [xc;yc];
    vec = M*vec;
    u = vec(1);
    v = vec(2);
end