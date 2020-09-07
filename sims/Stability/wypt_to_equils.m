step = 1;
x0s = -15:step:-1;
y0s = -30:step:-1;
x0s = [x0s -15:step:-1];
y0s = [y0s 1:step:15];
x0s = [x0s 1:step:15];
y0s = [y0s 1:step:15];
lambd = 10000;

wypts.xs = zeros(length(x0s)*length(y0s),1);
wypts.ys = zeros(length(x0s)*length(y0s),1);
equil.xs = zeros(length(x0s)*length(y0s),1);
equil.ys = zeros(length(x0s)*length(y0s),1);

options = optimoptions('fmincon','Display','off');

%% 
count = 1;
w = waitbar(0,"Calculating Equilibrium Points");
for j = 1:length(x0s)
   for i = 1:length(y0s)
       x0 = x0s(j);
       y0 = y0s(i);
       if x0>0 && y0<0
           continue;
       end
       x = optimvar('x',1,2); % x1 = x, x2 = y
%        cons1 = x(1) <= -0.01;
       cost = (x0 - x(1))^2 + (y0 - x(2))^2 + lambd*(atan(x(2)/x(1))/x(2))^2;
       prob = optimproblem('Objective',cost);
%        prob.Constraints.cons1 = cons1;
       x_start.x = [x0 y0];
       [sol,fval,exitflag,output] = solve(prob,x_start,'Options',options);
       x = sol.x(1);
       y = sol.x(2);
       equil.xs(count) = x;
       equil.ys(count) = y;
       wypts.xs(count) = x0;
       wypts.ys(count) = y0;
       count = count + 1;
       waitbar(count/(length(x0s)*length(y0s)),w,"Calculating Equilibrium Points");
   end
end
close(w);
%% Plot Equilibra
cmap = cool(length(x0s)*length(y0s));
gray = 0.5*ones(1,3);
figure(1);
subplot(1,2,1);
hold on;
scatter(wypts.xs,wypts.ys,5,cmap,'filled');
patch([0 x0s(end) x0s(end) 0],[y0s(1) y0s(1) 0 0],gray);
patch([1.1*x0s(1) x0s(1) x0s(1) 1.1*x0s(1)],[y0s(1),y0s(1),1.1*y0s(end),1.1*y0s(end)],gray);
patch([1.1*x0s(1) x0s(end) x0s(end) 1.1*x0s(1)],[y0s(end) y0s(end) 1.1*y0s(end) 1.1*y0s(end)],gray);
ylim([y0s(1),1.1*y0s(end)]);
xlim([1.1*x0s(1),x0s(end)]);
xlabel("X Pos (m)");
ylabel("Y Pos (m)");
title("Waypoint Placement");
subplot(1,2,2);
scatter(equil.xs,equil.ys,5,cmap,'filled');
patch([0 x0s(end) x0s(end) 0],[y0s(1) y0s(1) 0 0],gray);
patch([1.1*x0s(1) x0s(1) x0s(1) 1.1*x0s(1)],[y0s(1),y0s(1),1.1*y0s(end),1.1*y0s(end)],gray);
patch([1.1*x0s(1) x0s(end) x0s(end) 1.1*x0s(1)],[y0s(end) y0s(end) 1.1*y0s(end) 1.1*y0s(end)],gray);
ylim([y0s(1),1.1*y0s(end)]);
xlim([1.1*x0s(1),x0s(end)]);
xlabel("X Pos (m)");
ylabel("Y Pos (m)");
title("Equilibrium Point");
sgtitle("Mapping Waypoint Placement to Equilibrium Point");