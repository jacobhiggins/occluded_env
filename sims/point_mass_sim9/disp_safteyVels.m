close all;

hw1 = 30;
hw2 = 10;
hl1 = 100;
hl2 = 100;

x = 0.1:0.1:hw1;
y = 0.1:0.1:hl1-hw2+1;

[X,Y] = meshgrid(x,y);
xc = 0;
yc = 0;
xm2 = 0;
ym2 = hw2/2;
theta2 = 0;


ppm = 5; % pixel per meter

img_walls = ones((hl1+2)*ppm,(hl2+2)*ppm,3);
img_walls2 = ones((hl1+2)*ppm,(hl2+2)*ppm,3); % this is just to get the colorbar
img_walls(:,1:ppm,:) = 0; % Left wall of 1 m
img_walls(1:ppm,:,:) = 0; % Top wall of 1 m
img_walls(hw2*ppm+ppm:end,hw1*ppm+ppm:end,:) = 0; % Right Walls

cmap = jet(256);

img_vels = ones(size(img_walls));
for i = 1:length(x)
   for j = 1:length(y)
%        disp(i);
%        disp(j);
%        disp("");
       [v,vmax] = safety_vel(-x(i),-y(j),xc,yc,xm2,ym2,theta2,hw2);
%        v = 0.1;
%        v_max = 1;
       color = cmap(int16(256*v/vmax),:);
       img_walls2(hw2*ppm+ppm+int16(y(j)*ppm),hw1*ppm+ppm-int16(x(i)*ppm),:) = v/vmax; % Just to get colorbar
       img_walls(hw2*ppm+ppm+int16(y(j)*ppm),hw1*ppm+ppm-int16(x(i)*ppm),:) = cmap(int16(256*v/vmax),:);
       
   end
end

imshow(img_walls2,cmap); % Get colorbar
colorbar;
hold on;
imshow(img_walls); % Get actual plot
title("Velocity Profile");
xlabel("X Position");
ylabel("Y Position");
