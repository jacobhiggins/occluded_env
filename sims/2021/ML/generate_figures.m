% load("no_perc_no_safety.mat");
% load("perc_no_safety.mat");
load("safe_no_perc.mat");
close all;
wall_width = 0.5;
font_size = 13;

% Pure pursuit
x_ll = 1.35;
x_ul = 1.708;
y_ll = 6.3;
y_ul = 7.709;
% Perception, no safety
% x_ll = 0.2083;
% x_ul = 0.2345;
% y_ll = 6.114;
% y_ul = 6.501;
% Safe, no perception
x_ll = 1.814;
x_ul = 1.956;
y_ll = 6.444;
y_ul = 6.885;
%%
fig1 = figure(1);
hold on;

plt_model = plot(map.wypt_bases(1:3,1),map.wypt_bases(1:3,2),"r--","LineWidth",2,"DisplayName","Model Trajectory");
plt_trail = plot(robot.trail.x,robot.trail.y,"b--","LineWidth",2,"DisplayName","Actual Trajectory");

is = (robot.trail.x > x_ll).*(robot.trail.x < x_ul).*(robot.trail.y > y_ll).*(robot.trail.y < y_ul);
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
    plt_ku = fill(post_data.ku.polys(i).x,post_data.ku.polys(i).y,"r",...
        "DisplayName","Known-Unknown",...
        "FaceAlpha",0.5);
catch
    plt_ku = [];
end


patch([map.hws(1) map.hws(1)+wall_width map.hws(1)+wall_width map.hws(1)],[0 0 map.hls(1)-map.hws(2)*ones(1,2)],zeros(1,3));
patch([map.hws(1) 10 10 map.hws(1)],[(map.hls(1)-map.hws(2))*ones(1,2) (map.hls(1)-map.hws(2)-wall_width)*ones(1,2)],zeros(1,3));
patch([-wall_width 0 0 -wall_width],[0 0 10 10],zeros(1,3));
patch([-wall_width map.hls(2) map.hls(2) -wall_width],[map.hls(1) map.hls(1) (map.hls(1)+wall_width)*ones(1,2)],zeros(1,3));
axis equal;
xlim([-wall_width 10                                                                                                                                                                                                                                                                                                                                                        ]);
ylim([2.0 map.hls(1)+wall_width]);
lg = legend([plt_model,plt_trail,plt_robot,plt_heading,plt_ku],"Location","southeast","FontSize",font_size);
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
fig1.Position = [100 250 500 320];
%%
fig2 = figure(2);
plot(post_data.ts,post_data.ku.areas,"LineWidth",2);
xlabel("Time (s)");
ylabel("Known-unknown area (m^2)");
grid on;
xlim([0 6]);
ax = gca;
ax.XAxis.FontSize = font_size;
ax.YAxis.FontSize = font_size;
%%
fig2.Position = [500 250 650 320];


