%% Load Data
load("perception_no_safety.mat");
load("w_perception.mat");
p_no_s.trail = robot.trail;
p_no_s.ku = post_data.ku;
p_no_s.t = post_data.ts;
load("no_perception_no_safety.mat");
load("wout_perception.mat");
no_p_no_s.trail = robot.trail;
no_p_no_s.ku = post_data.ku;
no_p_no_s.t = post_data.ts;
close all;

hws = map.hws;
hls = map.hls;

map.walls{1}.x = map.walls{1}.x + [0 5 5 0];

%% Get Data for example point in time

tmp = logical(robot.trail.x < 3.9);

is = logical(robot.trail.x < 3.9) .* logical(robot.trail.x > 3.8) .* ...
    logical(robot.trail.y < 13.7) .* logical(robot.trail.y > 13.6);
is = find(logical(is));
index = is(1);

x = robot.trail.x(index);
y = robot.trail.y(index);
heading = [robot.vs.vxs(index);robot.vs.vys(index)];
heading = heading/norm(heading);

r = map.maxRad_suggest;
circ_points.x = x + r*cos(0:0.01:2*pi);
circ_points.y = y + r*sin(0:0.01:2*pi);

%% Plot trail
close all;
gray = 0.5*ones(1,3);
figure(1);
hold on;
for i = 1:length(map.walls)
   patch(map.walls{i}.x, map.walls{i}.y, gray); 
end

plt_no_p = plot(no_p_no_s.trail.x,no_p_no_s.trail.y,"LineWidth",2,"DisplayName","Motion without perception");
plt_p = plot(p_no_s.trail.x,p_no_s.trail.y,"LineWidth",2,"DisplayName","Motion with perception");
plt_model_traj = plot(map.wypt_bases(:,1),map.wypt_bases(:,2),"r--","DisplayName","Model Trajectory");

plt_r = plot(x,y,'ro','MarkerSize',10,"DisplayName","Robot");
plt_heading = quiver(x,y,5*heading(1),5*heading(2),"LineWidth",1,"DisplayName","Heading");
plt_ku = fill(robot.ku.polys(index).x,robot.ku.polys(index).y,"r","FaceAlpha",0.3,"DisplayName","Known-Unknown Area");
plt_FOV = plot(circ_points.x,circ_points.y,"DisplayName","FOV");

legend([plt_no_p,plt_p,plt_model_traj,plt_r,plt_heading,plt_ku,plt_FOV],"Location","southeast");
axis equal;
xlim([0,hls(2)]);
ylim([0,hls(1)+hls(3)-hws(2)-10]);
xticklabels({});
yticklabels({});