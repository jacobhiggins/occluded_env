vs_wperc = load("vs_wperc.mat"); % Velocities of pm with perception obj
vs_woutperc = load("vs_woutperc.mat"); % Velocities without perception obj



close all;
figure(1);
hold on;
vs1 = vs_wperc.vs;
vs2 = vs_woutperc.vs;
plot(0.1*(1:length(vs1)),vs1(1,:),"r-","LineWidth",2,"DisplayName","With Perc., X Vel");
plot(0.1*(1:length(vs1)),vs1(2,:),"r--","LineWidth",2,"DisplayName","With Perc., Y Vel");
plot(0.1*(1:length(vs2)),vs2(1,:),"b-","LineWidth",2,"DisplayName","Without Perc., X Vel");
plot(0.1*(1:length(vs2)),vs2(2,:),"b--","LineWidth",2,"DisplayName","Without Perc., Y Vel");
title("Comparing Velocities with/without Perception Objective");
legend("Location","eastoutside");
figure(2);
hold on;
plot(0.1*(1:length(vs1)),sqrt(vs1(1,:).^2 + vs1(2,:).^2),"DisplayName","with perc");
plot(0.1*(1:length(vs2)),sqrt(vs2(1,:).^2 + vs2(2,:).^2),"DisplayName","without perc");
legend;