syms cost(x,x0,y,y0)
cost(x,y) = (x-x0)^2 + (y-y0)^2 + (atan(y/x)/y)^2;
Dfx = diff(cost,x);
Dfy = diff(cost,y);
Dfxx = diff(Dfx,x);
Dfyy = diff(Dfy,y);
Dfxy = diff(Dfx,y);