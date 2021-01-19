% Load data
close all;
data = load("TIntersect_robotup_trafficdown_wperc.mat");
ru_td_wp = data.robot;
data = load("TIntersect_robotup_trafficup_wperc.mat");
ru_tu_wp = data.robot;
data = load("TIntersect_robotup_trafficdown_woutperc.mat");
ru_td_woutp = data.robot;
data = load("TIntersect_robotup_trafficup_woutperc.mat");
ru_tu_woutp = data.robot;

%% Plotting robot up, traffic down with/without perception control framework
figure(1);
subplot(2,1,1);
hold on;
plot(ru_td_woutp.dc.times,ru_td_woutp.dc.vels.norm,...
    "LineWidth",2);
plot(ru_td_wp.dc.times,ru_td_wp.dc.vels.norm,...
    "LineWidth",2);
legend(["Without Control Framework","With Control Framework"]);
ylabel("Speed (m/s)");
grid on;
subplot(2,1,2);
hold on;
plot(ru_td_woutp.dc.times,ru_td_woutp.dc.KUs.area,...
    "LineWidth",2);
plot(ru_td_wp.dc.times,ru_td_wp.dc.KUs.area,...
    "LineWidth",2);
ylabel("Known unknown area (m^2)");
xlabel("Time (s)");
sgtitle("Robot moving up, traffic moving down");
grid on;
%% Plotting robot up, traffic up with/without perception control framework
figure(2);
subplot(2,1,1);
hold on;
plot(ru_tu_woutp.dc.times,ru_tu_woutp.dc.vels.norm,...
    "LineWidth",2);
plot(ru_tu_wp.dc.times,ru_tu_wp.dc.vels.norm,...
    "LineWidth",2);
legend(["Without Control Framework","With Control Framework"]);
ylabel("Speed (m/s)");
grid on;
subplot(2,1,2);
hold on;
plot(ru_tu_woutp.dc.times,ru_tu_woutp.dc.KUs.area,...
    "LineWidth",2);
plot(ru_tu_wp.dc.times,ru_tu_wp.dc.KUs.area,...
    "LineWidth",2);
ylabel("Known unknown area (m^2)");
xlabel("Time (s)");
sgtitle("Robot moving up, traffic moving up");
grid on;