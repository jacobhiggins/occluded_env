close all;
x = 0.1:0.1:35;
y = 0.1:0.1:35;
[X,Y] = meshgrid(x,y);
Ds = sqrt(X.^2 + Y.^2);
theta = abs(atan(Y./X));
phi_norm = 1 - atan(Y./X)/(pi/2);
vel2 = @(D,phi_norm) (0.5 + (max(phi_norm*10,0.5) - 0.5)*D/35).*(D<35) + 10*(D>=35);

% New velocity function
vf = 10;
vs = 0.5;
D = 35;
A = [1 0 0 0;1 D D^2 D^3;0 1 0 0;0 1 2*D 3*D^2];
b = [vs;vf;0;0];
c = A\b;
vel3 = @(d) ([ones(size(d)),d,d.^2,d.^3]*c).*(d<=35) + vf*(d>35);
phi3 = @(theta) ((theta - pi/4*ones(size(theta))).^2)./((theta-pi/4*ones(size(theta))).^2 + (10*pi/180)^2*ones(size(theta)));

vels = zeros(size(Ds));
for i = 1:size(Ds,1)
   for j = 1:size(Ds,2)
%       vels(i,j) = vel2(Ds(i,j),phi_norm(i,j)); 
%         vels(i,j) = max(vel3(Ds(i,j))*phi3(theta(i,j)),vs);
        vels(i,j) = set_traj_vel2(Ds(i,j),35,phi_norm(i,j),0);
   end
end
hm = heatmap(flip(vels,2));
hm.GridVisible = 'off';
labels = repmat({''},size(Ds,1),1);
set(gca,'XDisplayLabels',labels);
set(gca,'YDisplayLabels',labels);
xlabel("X Position (m)");
ylabel("Y Position (m)");
title("Set Velocity vs Position");
figure(2);
thetas = 0:0.1:pi/2;
distances= 0:0.1:50;
data = zeros(1,length(distances));
for i = 1:length(distances)
    d = distances(i);
    data(i) = set_traj_vel2(d,35,0.9,1);
end
plot(distances,data);
xlabel("Distance (m)");
ylabel("Set Velocity (m)");
title("Pursuit Velocity vs. Distance to Corner");
ylim([0,12]);