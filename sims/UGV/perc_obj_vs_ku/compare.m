% x-y data points
hw1 = -15;
hl1 = -30;
hw2 = 15;

xs = hw1:0.1:-0.1;
ys = hl1:0.1:-0.1;

p.maxRad = 10;
p.next_owall = hw2;

data = struct("ku",[],"po",[]);

data.ku = zeros(length(ys),length(xs));
data.po = zeros(length(ys),length(xs));

%% Calculate perception objective and known unknown area for each point
for i = 1:length(xs)
   for j = 1:length(ys)
      p.x = xs(i);
      p.y = ys(j);
      ku = knownunknown(p);
      po = perc_obj(p);
      data.ku(length(ys)-j+1,i) = ku.area;
      data.po(length(ys)-j+1,i) = po;
   end
end

%% Process and plot data
is = abs(data.ku)>0.0001;
data.ku = data.ku(is);
data.po = data.po(is);

close all;
figure(1);
plot(data.ku(1:100),data.po(1:100));