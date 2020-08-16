%% Define constants
close all;
global hw1;
global hl1;
global hw2;
global radMax;
hw1 = 30;
hl1 = 30;
hw2 = 30;
radMax = 31;

%% Initialize Inputs / Outputs
x = -hw1:0.1:0;
y = -hl1:0.1:0;

[X,Y] = meshgrid(x,y);

vis = visFunction(X,Y);

%% Train NN for Function Approximation

xnn = vis.xnn;
ynn = vis.ynn;
% znn = cell2mat(reshape(vis.polygon,[4,length(xnn)]));
data = [xnn ynn];
data = data(randperm(size(data,1)),:);
xnn = data(:,1:2);
ynn = data(:,3);

%% Visualize Area
% Robot placement
close all;
i = 100;
xr = -25;
yr = -5;
perc = ynn(i);

figure(1);
hold on;
% Get polygon points
xi = find(x==xr);
yi = find(y==yr);
points = vis.polygon{yi,xi};
plt_r = plot(xr,yr,'rx',"DisplayName","Robot Location");
plt_corners = plot(points(:,1),points(:,2),'bo','MarkerSize',5,"DisplayName","Corners of Unknown");
plt_area = fill(points(:,1),points(:,2),'r',"DisplayName","Unknown Area");

fprintf("Area of Unknown: %d\n",vis.Data{yi,xi});
fprintf("NN2 approximation of Unknown: %f\n",visNN2([xr,yr]));

axis equal;
xlim([-hw1,radMax]);
ylim([-hl1,hw2+10]);
gray = [0.5,0.5,0.5];
patch([0,radMax,radMax,0],[-hl1,-hl1,0,0],gray);
patch([-hw1,radMax,radMax,-hw1],[hw2,hw2,hw2+10,hw2+10],gray);
circ = circ_points(xr,yr,radMax);
plot(circ.x,circ.y,'-');
legend([plt_r,plt_corners,plt_area],"Location","eastoutside");
title("Visualizing Unknown Area");


