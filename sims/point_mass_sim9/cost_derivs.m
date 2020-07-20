syms cost(x,x0,y,y0,c)
cost(x,y) = (x-x0)^2 + (y-y0)^2 + c*(atan(y/x)/y)^2;
cost_dx = diff(cost,x);
cost_dy = diff(cost,y);
cost_ddx = diff(cost_dx,x);
cost_ddy = diff(cost_dy,y);
cost_dxdy = diff(cost_dx,y);
H = [cost_ddx cost_dxdy;
    cost_dxdy cost_ddy];
lambda = eig(H);

cost_dx_handle = matlabFunction(cost_dx);
cost_dy_handle = matlabFunction(cost_dy);
cost_ddx_handle = matlabFunction(cost_ddx);
cost_ddy_handle = matlabFunction(cost_ddy);
cost_dxdy_handle = matlabFunction(cost_dxdy);
H_handle = matlabFunction(H);

%% Solve for Zeros

% Note that you can't find an explicit solution to zero crossing of derivs
% since they're transcendental equations!

% x = fzero(cost_dx_handle,[-30,0]);
c = 10000;
x0 = -5;
y0 = -30;
% Test newton's method
[x,y] = newtonsMethod(x0,y0,c,cost_dx_handle,cost_dy_handle,H_handle);

%% Plot equilibrium points

x0 = -30:1:-1; % Location of Waypoint
y0 = -30:1:-1; % Location of Waypoint
c = 10000; % Perception objective weight
wypts.xs = zeros(length(x0)*length(y0),1);
wypts.ys = zeros(length(x0)*length(y0),1);
equil.xs = zeros(length(x0)*length(y0),1);
equil.ys = zeros(length(x0)*length(y0),1);
count = 1;
w = waitbar(0,"Calculating Equilibrium Points");
for j = 1:length(x0)
   for i = 1:length(y0)
       [x,y] = newtonsMethod(x0(j),y0(i),c,cost_dx_handle,cost_dy_handle,H_handle);
       equil.xs(count) = x;
       equil.ys(count) = y;
       wypts.xs(count) = x0(j);
       wypts.ys(count) = y0(i);
       count = count + 1;
       waitbar(count/(length(x0)*length(y0)),w,"Calculating Equilibrium Points");
   end
end
close(w);
cmap = cool(length(x0)*length(y0));
figure(1);
subplot(1,2,1);
scatter(wypts.xs,wypts.ys,5,cmap,'filled');
ylim([y0(1),0]);
xlim([x0(1),0]);
xlabel("X Pos (m)");
ylabel("Y Pos (m)");
title("Waypoint Placement");
subplot(1,2,2);
scatter(equil.xs,equil.ys,5,cmap,'filled');
ylim([y0(1),0]);
xlim([x0(1),0]);
xlabel("X Pos (m)");
ylabel("Y Pos (m)");
title("Equilibrium Point");
sgtitle("Mapping Waypoint Placement to Equilibrium Point");

%% Plot functions

xs = -30:0.1:-0.1;
ys = -30:0.1:-0.1;

[X,Y] = meshgrid(xs,ys);

data = cost_ddx_handle(X,Y,1);
s = surf(xs,ys,data);
s.EdgeColor = 'none';