close all;
load("LOS_probs.mat");
rs = p_prob.rad_probs.r;
probs = p_prob.rad_probs.prob;
% Sort
% [rs,I] = sort(rs);
% probs = probs(I);
% Find cdf
cum_probs = zeros(1,length(probs));
cum_prob = 0.0;
for i = 1:length(probs)
    cum_prob = cum_prob + probs(i);
    cum_probs(i) = cum_prob;
end
% plot
subplot(1,2,1);
plot(rs,probs);
title("PDF");
xlabel("Distance (m)");
ylabel("Probability Density");
subplot(1,2,2);
plot(rs,cum_probs);
xlabel("Distance (m)");
ylabel("Probability");
title("CDF");
sgtitle("Distance Between Robot and Dynamic Obstacle at First Sight");