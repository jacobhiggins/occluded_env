load("no_uncertainty.mat");
robot_no_unc = robot;
post_data_no_unc = post_data;
map_no_unc = map;
load("with_uncertainty.mat");
robot_w_unc = robot;
post_data_w_unc = post_data;
map_w_unc = map;
close all;

%% Compare Trajectories
figure(1);
hold on;
black = 0.0*ones(1,3);
white = ones(1,3);
for i = 1:length(map.walls)
    wall = map.walls{i};
    patch(wall.x,wall.y,black);
end
plot(robot_no_unc.trail.x,robot_no_unc.trail.y,"-",...
    "LineWidth",3);
plot(robot_w_unc.trail.x,robot_w_unc.trail.y,"-.",...
    "LineWidth",3);
axis equal;
xlim([-1 21]);
ylim([0 38]);
set(gca,"FontSize",15);
%% Compare Known-unknows + velocities
figure(2);
% subplot(2,1,1);
hold on;
plot(post_data_no_unc.ts,post_data_no_unc.kus.areas,"-",...
    "DisplayName","Safe in Main Hallway",...
    "LineWidth",2);
plot(post_data_w_unc.ts,post_data_w_unc.kus.areas,"-.",...
    "DisplayName","Uncertain in Main Hallway",...
    "LineWidth",2);
lg = legend("FontSize",12);
xlabel("Time (s)","FontSize",15);
ylabel("A_{ku} (m^2)","FontSize",15);
set(gca,"FontSize",15);
figure(3);
hold on;
plot(post_data_no_unc.ts,post_data_no_unc.vs,...
    "DisplayName","No Uncertainty");
plot(post_data_w_unc.ts,post_data_w_unc.vs,...
    "DisplayName","With Uncertainty");
legend();
%% Hallway Setup