load("stats.mat");
close all;
figure(1);
hold on;
subplot(2,1,1);
semilogx([stats.perception_weight],[stats.time2clear],"LineWidth",2);
ylabel("Time to clear corner (s)");
subplot(2,1,2);
semilogx([stats.perception_weight],[stats.max_KU],"LineWidth",2);
ylabel("Maximum KU area");
xlabel("Perception Weight");