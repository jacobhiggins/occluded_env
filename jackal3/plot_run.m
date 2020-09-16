%% Load Data
load("robot.mat");
ts = robot.ts;
vs = robot.vs;
trail = robot.trail;
proj_mots = robot.proj_mots;

%% Plot data, all at once
figure(1);
plot(trail.x,trail.y);
axis equal;
grid on;
xlabel("X Position (m)");
ylabel("Y Position (m)");

%% Show vid, plotting data over time
figure(2);
delts = (ts(2:end) - ts(1:end-1))*10^-9;
xs = [trail.x(1)];
ys = [trail.y(1)];
vid = VideoWriter("vid.avi");
hold on;
plt_r = plot(trail.x(1),trail.y(1),"o","MarkerSize",50,"DisplayName","Jackal3");
plt_wypt = plot(robot.wypt.x,robot.wypt.y,"x","MarkerSize",10,"DisplayName","Waypoint");
head_scale = 0.5;
plt_heading = quiver(trail.x(1),...
    trail.y(1),...
    head_scale*cos(trail.theta(1)),...
    head_scale*sin(trail.theta(1)),...
    "DisplayName","Heading",...
    "LineWidth",2);
plt_proj = plot(proj_mots(1).x,proj_mots(1).y,...
    "DisplayName","Projected Motion",...
    "LineWidth",2);
plt_trail = plot(xs,ys,"DisplayName","Trail",...
    "LineWidth",2);
xlim([-2.5,robot.wypt.x*1.1+0.1]);
ylim([-2.5,robot.wypt.y*1.1+0.1]);
axis equal;
grid on;
xlabel("X Position (m)");
ylabel("Y Position (m)");
title("Experiments");
legend([plt_r,plt_proj],"Location","eastoutside");
open(vid);
for i = 1:length(delts)
    plt_r.XData = trail.x(i);
    plt_r.YData = trail.y(i);
    plt_heading.XData = trail.x(i);
    plt_heading.YData = trail.y(i);
    plt_heading.UData = head_scale*cos(trail.theta(i));
    plt_heading.VData = head_scale*sin(trail.theta(i));
    plt_proj.XData = proj_mots(i).x;
    plt_proj.YData = proj_mots(i).y;
    xs = [xs trail.x(i)];
    ys = [ys trail.y(i)];
    plt_trail.XData = xs;
    plt_trail.YData = ys;
    frame = getframe(gcf);
    writeVideo(vid,frame);
    pause(delts(i));
end
close(vid);