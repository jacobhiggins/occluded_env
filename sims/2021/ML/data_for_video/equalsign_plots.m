close all;
% Load runs of maps
w_perc = load("equalsign_wperc.mat");
w_perc = w_perc.robot.dc;
wout_perc = load("equalsign_woutperc.mat");
wout_perc = wout_perc.robot.dc;

% Plot KU area for two runs against each other
figure(1);
hold on;
plot(w_perc.times,w_perc.KUs.area,...
    "LineWidth",2);
plot(wout_perc.times,wout_perc.KUs.area,...
    "LineWidth",2);
legend(["With perception","Without perception"]);
xlabel("Time (s)");
ylabel("Known-Unknown area");
grid on;