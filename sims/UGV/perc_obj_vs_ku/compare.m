% x-y data points
hw1 = 15;
hl1 = 30;
hw2 = 15;

xs = [-hw1:0.05:-0.7,-0.7:0.005:-0.1];
ys = [-hl1:0.05:-0.7,-0.7:0.005:-0.1];

p.maxRad = 10;
p.next_owall = hw2;

data = struct("ku",[],"po",[]);

data.ku = zeros(length(ys),length(xs));
data.po = zeros(length(ys),length(xs));
data.x = zeros(length(ys),length(xs));
data.y = zeros(length(ys),length(xs));

%% Calculate perception objective and known unknown area for each point
for i = 1:length(xs)
   for j = 1:length(ys)
      p.x = xs(i);
      p.y = ys(j);
      ku = knownunknown(p);
      po = perc_obj(p);
      data.ku(length(ys)-j+1,i) = ku.area;
      data.po(length(ys)-j+1,i) = po;
      data.x(length(ys)-j+1,i) = p.x;
      data.y(length(ys)-j+1,i) = p.y;
   end
end

save("data.mat","data");

%% Process data

load("data.mat");
is = abs(data.ku)>0.0001;
data.ku = data.ku(is);
data.po = log(data.po(is));
data.x = data.x(is);
data.y = data.y(is);

angle_is = logical( ((-3*pi/4 - 0.0005) <= atan2(data.y,data.x)) .* (atan2(data.y,data.x) <= (-3*pi/4 + 0.0005)) );
data_copy = data;

% Rearrange data by radius or angle
radius = (data.x.^2 + data.y.^2) + 300*atan2(data.y,data.x);
[r_sorted,I] = sort(radius);
data.ku = data.ku(I);
data.po = data.po(I);
data.x = data.x(I);
data.y = data.y(I);

%% Plot Data


% num_data = 7000;
num_data = length(data.ku);

cmap = winter(num_data);

close all;
figure(1);
% subplot(1,2,2);
hold on;
scatter(data.ku(1:num_data),data.po(1:num_data),20,cmap(1:num_data,:),"filled");
scatter(data_copy.ku(angle_is),data_copy.po(angle_is),10,'r','filled');
xlabel("A_{ku} (m^2)","FontSize",16);
ylabel("ln(\Lambda(x,y))","FontSize",16);
% grid on;
% close all;
figure(2);
hold on;
% subplot(1,2,2);
gray = 0.5*ones(1,3);
hold on;
patch([0 hw2 hw2 0],[-hl1 -hl1 0 0],gray);
scatter(data.x(1:num_data),data.y(1:num_data),8,cmap(1:num_data,:),"filled");
scatter(data_copy.x(angle_is),data_copy.y(angle_is),8,'r','filled');
axis equal;
xlim([-p.maxRad*1.1 1.1]);
ylim([-p.maxRad*1.1 1.1]);
xlabel("X pos (m)","FontSize",16);
ylabel("Y pos (m)","FontSize",16);
% sgtitle("Mapping position to known-unknown");

%% Plot correlations of angles of points
corrs = [];
angles = -pi/2:-0.02:-pi;
for angle = angles
    angles_is = logical( ((angle - 0.02) <= atan2(data_copy.y,data_copy.x)) .* (atan2(data_copy.y,data_copy.x) <= (angle + 0.02)) );
    corr = corrcoef(data_copy.ku(angles_is),data_copy.po(angles_is));
    corrs = [corrs corr(2,1)];
end

figure(2);
plot(angles,corrs);
