x = -10:0.01:10;
y = -10:0.01:10;

[X,Y] = meshgrid(x,y);

%% Vis cost 2D
close all;
xw = -5;
yw = -1;

[cs,cs_wypt,cs_perc] = cost2D(x,yw,xw,100);

figure(1);
subplot(2,1,1);
hold on;
plot(x,cs_wypt,"DisplayName","Waypoint Objective");
plot(x,cs_perc,"DisplayName","Perception Objective");
xlim([-10,10]);
ylim([0,100]);
legend;
grid on;
title("Objective Function Components");
subplot(2,1,2);
plot(x,cs);
xlim([-10,10]);
ylim([0,100]);
grid on;
title("Total Cost");
% zlim([0,100]);