% Plot the difference in velocities between no perception and perception
% objectives included in the MPC
close all;
% vs_1 = load("vs_control_woutp.mat");
data_woutp = load("data_woutp.mat");
data_wp = load("data_wp.mat");
ts_woutp = data_woutp.post_data.ts;
ts_wp = data_wp.post_data.ts;
vs_woutp = data_woutp.post_data.vs;
vs_wp  = data_wp.post_data.vs;
LOSs_woutp = data_woutp.post_data.LOSs;
LOSs_wp = data_wp.post_data.LOSs;

figure(1);
subplot(2,1,1);
hold on;
plot(ts_woutp,vs_woutp);
plot(ts_wp,vs_wp);
ylabel("Velocity (m/s)");
title("Velocity Comparison");
legend("Without Perception","With Perception","Location","eastoutside");
grid on;
subplot(2,1,2);
hold on;
plot(ts_woutp,LOSs_woutp);
plot(ts_wp,LOSs_wp);
xlabel("time (s)");
ylabel("Line of Sight (m)");
title("Line of Sight Comparison");
legend("Without Perception","With Perception","Location","eastoutside");
grid on;