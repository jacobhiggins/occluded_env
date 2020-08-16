close all;
load("../robot.mat");
LOS = robot.rs;
ku_area = robot.ku.areas;
ku_area_proc = ku_area(ku_area>0.001);
LOS_proc = LOS(ku_area>0.001);
cmap = hsv(length(LOS_proc));
figure;
hold on;
for i = 1:length(LOS_proc)-1
    plot([LOS_proc(i),LOS_proc(i+1)],[ku_area_proc(i),ku_area_proc(i+1)],"LineWidth",2,"Color",cmap(i,:));
end
xlabel("LOS (m)");
ylabel("KU Area (m^2)");