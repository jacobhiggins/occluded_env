syms cost(x,x0,y,y0)
lambd = 2500;
cost(x,y) = (x-x0)^2 + (y-y0)^2 + lambd*(atan(y/x)/y)^2;

Dfx = diff(cost,x);
Dfy = diff(cost,y);
% Dfxx = diff(Dfx,x);
% Dfyy = diff(Dfy,y);
% Dfxy = diff(Dfx,y);

xs = -10:0.5:-0.1;
ys = -10:0.5:-0.1;

[X,Y] = meshgrid(xs,ys);

x0 = -5;
y0 = -5;


%% First, find equilibrium point

options = optimoptions('fmincon','Display','off');
r = optimvar('x',1,2); % x1 = x, x2 = y
cost = (x0 - r(1))^2 + (y0 - r(2))^2 + lambd*(atan(r(2)/r(1))/r(2))^2;
prob = optimproblem('Objective',cost);
r_start.x = [x0 y0];
[sol,fval,exitflag,output] = solve(prob,r_start,'Options',options);
x_equil = sol.x(1);
y_equil = sol.x(2);

%% Next, show that moving towards equilibrium always decreases cost

data = ones(size(X));

count = 1;
w = waitbar(0,"Calculating Equilibrium Points");
for i = 1:size(X,1)
   for j = 1:size(X,2)
      x = X(i,j);
      y = Y(i,j);
      grad = [double(subs(Dfx)),double(subs(Dfy))];
      vec = [x_equil-x;y_equil-y];
      vec = vec/norm(vec);
      value = grad*vec;
      data(i,j) = value;
      count = count + 1;
      waitbar(count/(length(xs)*length(ys)),w,"Calculating Equilibrium Points");
   end
end
close(w);

save("data.mat","data","X","Y");

%% Process Data

min_z = -2000;

load("data.mat");
is = data > min_z;
data_proc = data(is);
X_proc = X(is);
Y_proc = Y(is);

%% Plot data
close all;

T_proc = delaunay(X_proc,Y_proc);
T= delaunay(X,Y);

figure(1);
hold on;
% surf(X,Y,data,"EdgeColor","none");
% surf(X,Y,0*data,C,"EdgeColor","none","FaceAlpha",0.4);
trisurf(T_proc,X_proc,Y_proc,-log(-data_proc+1),'EdgeColor','none');
% set(gca,'zscale','log');
trisurf(T,X,Y,zeros(size(data)),"EdgeColor","none","FaceAlpha",0.4,"FaceColor","g");
plot3([x0 x0],[y0 y0],[1.2*min(-log(-data_proc+1)) 5],"LineWidth",4);
plot3(x_equil*ones(1,2), y_equil*ones(1,2), [1.2*min(-log(-data_proc+1)) 5], "LineWidth", 4);
text(x0,y0,6,"Waypoint");
text(x_equil,y_equil,6,"Equilibrium");
view(45,45);
xlabel("X Position (m)");
ylabel("Y Position (m)");
zlabel("-ln (-\nablaJ_{pos} \cdot (p_{eq} - p) + 1)");
% zlim([-20 0]);