load("no_perc_no_safety.mat");
robot_pp = robot;
map_pp = map;
post_data_pp = post_data;
load("perc_no_safety.mat");
robot_perc = robot;
map_perc = map;
post_data_perc = post_data;
load("safe_no_perc.mat");
robot_safety = robot;
map_safety = map;
post_data_safety = post_data;
close all;
wall_width = 0.5;
font_size = 13;

% Pure pursuit
x_ll = 1.35;
x_ul = 1.708;
y_ll = 6.3;
y_ul = 7.709;
% Perception, no safety
x_ll_perc = 0.2083;
x_ul_perc = 0.2345;
y_ll_perc = 6.114;
y_ul_perc = 6.501;
% Safe, no perception
x_ll_safe = 1.814;
x_ul_safe = 1.956;
y_ll_safe = 6.444;
y_ul_safe = 6.885;
%%
fig1 = figure(1);
hold on;

plt_model = plot(map_perc.wypt_bases(1:3,1),map_perc.wypt_bases(1:3,2),"r--","LineWidth",2,"DisplayName","Model Trajectory");
plt_trail = plot(robot_perc.trail.x,robot_perc.trail.y,"b--","LineWidth",2,"DisplayName","Actual Trajectory");
plt_trail_pp = plot(robot_pp.trail.x,robot_pp.trail.y,"b--","LineWidth",2,"DisplayName","Pure Pursuit");

is = (robot_perc.trail.x > x_ll_perc).*(robot_perc.trail.x < x_ul_perc).*(robot_perc.trail.y > y_ll_perc).*(robot_perc.trail.y < y_ul_perc);
i = find(is,1);
scalar = 2;
plt_heading = quiver(robot_perc.trail.x(i),robot_perc.trail.y(i),scalar*robot_perc.cmd_inputs.x(i),scalar*robot_perc.cmd_inputs.y(i),...
    "DisplayName","Heading",...
    "LineWidth",3);
plt_robot = scatter(robot_perc.trail.x(i),robot_perc.trail.y(i),...
    "DisplayName","Robot",...
    "MarkerFacecolor","b",...
    "SizeData",200);
try
    plt_ku = fill(post_data_perc.ku.polys(i).x,post_data_perc.ku.polys(i).y,"r",...
        "DisplayName","Known-Unknown",...
        "FaceAlpha",0.5);
catch
    plt_ku = [];
end

patch([map_perc.hws(1) map_perc.hws(1)+wall_width map_perc.hws(1)+wall_width map_perc.hws(1)],[0 0 map_perc.hls(1)-map_perc.hws(2)*ones(1,2)],zeros(1,3));
patch([map_perc.hws(1) 10 10 map_perc.hws(1)],[(map_perc.hls(1)-map_perc.hws(2))*ones(1,2) (map_perc.hls(1)-map_perc.hws(2)-wall_width)*ones(1,2)],zeros(1,3));
patch([-wall_width 0 0 -wall_width],[0 0 10 10],zeros(1,3));
patch([-wall_width map_perc.hls(2) map_perc.hls(2) -wall_width],[map_perc.hls(1) map_perc.hls(1) (map_perc.hls(1)+wall_width)*ones(1,2)],zeros(1,3));
axis equal;
xlim([-wall_width 10                                                                                                                                                                                                                                                                                                                                                        ]);
ylim([2.0 map_perc.hls(1)+wall_width]);
lg = legend([plt_model,plt_trail,plt_trail_pp,plt_robot,plt_heading,plt_ku,],"Location","southeast","FontSize",font_size);
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
fig1.Position = [100 250 500 320];
%%
fig2 = figure(2);
hold on;
plot(post_data_perc.ts,post_data_perc.ku.areas,"LineWidth",2,...
    "DisplayName","With Perception");
plot(post_data_pp.ts,post_data_pp.ku.areas,"LineWidth",2,...
    "DisplayName","Pure Pursuit");

xlabel("Time (s)");
ylabel("A_{ku} (m^2)");
grid on;
xlim([0 6]);
legend();
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
%%
fig2.Position = [500 250 650 320];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55

%%
fig3 = figure(3);
hold on;

plt_model = plot(map_safety.wypt_bases(1:3,1),map_safety.wypt_bases(1:3,2),"r--","LineWidth",2,"DisplayName","Model Trajectory");
plt_trail = plot(robot_safety.trail.x,robot_safety.trail.y,"b--","LineWidth",2,"DisplayName","Actual Trajectory");
plt_trail_pp = plot(robot_pp.trail.x,robot_pp.trail.y,"b--","LineWidth",2,"DisplayName","Pure Pursuit");

is = (robot_safety.trail.x > x_ll_safe).*(robot_safety.trail.x < x_ul_safe).*(robot_safety.trail.y > y_ll_safe).*(robot.trail.y < y_ul_safe);
i = find(is,1);
scalar = 2;
plt_heading = quiver(robot.trail.x(i),robot.trail.y(i),scalar*robot.cmd_inputs.x(i),scalar*robot.cmd_inputs.y(i),...
    "DisplayName","Heading",...
    "LineWidth",3);
plt_robot = scatter(robot.trail.x(i),robot.trail.y(i),...
    "DisplayName","Robot",...
    "MarkerFacecolor","b",...
    "SizeData",200);
try
    plt_ku = fill(post_data_safety.ku.polys(i).x,post_data_safety.ku.polys(i).y,"r",...
        "DisplayName","Known-Unknown",...
        "FaceAlpha",0.5);
catch
    plt_ku = [];
end

patch([map_safety.hws(1) map_safety.hws(1)+wall_width map_safety.hws(1)+wall_width map_safety.hws(1)],[0 0 map_safety.hls(1)-map_safety.hws(2)*ones(1,2)],zeros(1,3));
patch([map_safety.hws(1) 10 10 map_safety.hws(1)],[(map_safety.hls(1)-map_safety.hws(2))*ones(1,2) (map_safety.hls(1)-map_safety.hws(2)-wall_width)*ones(1,2)],zeros(1,3));
patch([-wall_width 0 0 -wall_width],[0 0 10 10],zeros(1,3));
patch([-wall_width map_safety.hls(2) map_safety.hls(2) -wall_width],[map_safety.hls(1) map_safety.hls(1) (map_safety.hls(1)+wall_width)*ones(1,2)],zeros(1,3));
axis equal;
xlim([-wall_width 10                                                                                                                                                                                                                                                                                                                                                        ]);
ylim([2.0 map_safety.hls(1)+wall_width]);
lg = legend([plt_model,plt_trail,plt_trail_pp,plt_robot,plt_heading,plt_ku],"Location","southeast","FontSize",font_size);
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
fig3.Position = [100 250 500 320];
%%
fig4 = figure(4);
hold on;

plot(post_data_safety.ts,post_data_safety.ku.areas,"LineWidth",2,...
    "DisplayName","Safe");
plot(post_data_perc.ts,post_data_perc.ku.areas,"LineWidth",2,...
    "DisplayName","With Perception");
xlabel("Time (s)");
ylabel("A_{ku} (m^2)");
grid on;
xlim([0 6]);
legend();
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
%%
fig4.Position = [500 250 650 320];




