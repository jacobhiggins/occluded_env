clear MPCcost;
clear MPCcost_wper;
cost_func = @MPCcost;
params(); % Set global parameter values

% tic;
% PSO(cost,feas);
% toc;

% num = MPCcost(repmat(zeros(2,1),1,5),repmat([0.5;-0.2],4,5));

%% Run simple MPC simulation

T = 100;
global dt;
n = int16(T/dt);
x = [-15;-100];
xs = zeros(2,n+1);
xs(:,1) = x;
u = zeros(2,1);
us = zeros(2,n);
ts = zeros(1,n);
global A;
global B;
w = waitbar(0,'Computing MPC Trajectory');
cost_func = @MPCcost_wper;
for i = 1:n
   t = i*dt;
   u = PSO(x,cost_func);
   u_0 = u(1:2); % Only take first step
   x = A*x + B*u_0;
   ts(i) = t;
   xs(:,i+1) = x;
   us(:,i) = u_0;
   waitbar(double(t)/T,w,'Computing MPC Trajectory');
end
close(w);

%% Plot trajectory
close all;
global ref;
figure(1);
hold on;
gray = [0.5 0.5 0.5];
patch([0,30,30,0],[-100,-100,0,0],gray);
plt_r = plot(xs(1,:),xs(2,:),'-ob','DisplayName','robot trajectory');
plt_g = plot(ref(1),ref(2),'rx','MarkerSize',10,'DisplayName','goal');

axis equal;
xlim([-30,30]);
ylim([-100,30]);
legend([plt_r,plt_g],"Location","eastoutside");
title("Robot Motion with NN Perception");