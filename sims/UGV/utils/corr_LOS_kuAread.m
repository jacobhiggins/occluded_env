close all;
load("../robot.mat");
LOS = robot.rs;
ku_area = robot.ku.areas;
ku_area_proc = ku_area(ku_area>0.001);
LOS_proc = LOS(ku_area>0.001);
%% Plot with colors
cmap = hsv(length(LOS_proc));
figure;
hold on;
for i = 1:length(LOS_proc)-1
    plot([LOS_proc(i),LOS_proc(i+1)],[ku_area_proc(i),ku_area_proc(i+1)],"LineWidth",2,"Color",cmap(i,:));
end
xlabel("LOS (m)");
ylabel("KU Area (m^2)");
%% Make video of plot
v = VideoWriter("LOS_KU.avi","Uncompressed AVI");
open(v);
close all;
figure;
cmap = cool(length(LOS_proc));
hold on;
grid on;
plt = plot([LOS_proc(1),LOS_proc(2)],[ku_area_proc(1),ku_area_proc(2)],"LineWidth",2);
plt_head = plot(LOS_proc(1),ku_area_proc(2),'ro',"MarkerSize",10);
xlabel("LOS (m)");
ylabel("KU Area (m^2)");
xlim([0.98*min(LOS_proc),1.02*max(LOS_proc)]);
ylim([0,1.1*max(ku_area_proc)]);
title("Comparing Line of Sight to Known-Unknown Area");
for i = 1:length(LOS_proc)-1
    hold on;
    plt.XData = LOS_proc(1:i);
    plt.YData = ku_area_proc(1:i);
    plt_head.XData = LOS_proc(i);
    plt_head.YData = ku_area_proc(i);
%     plot(LOS_proc(1:i),ku_area_proc(1:i),"LineWidth",2)
    drawnow;
    frame = getframe(gcf);
    writeVideo(v,frame);
    pause(0.1);
end
close(v);