% clear all;
close all;
load("stats.mat");

% Plot when vx crosses zero twice
hallwidths = unique([stats.hallwidth]);
perception_weights = unique([stats.perception_weight]);
T = array2table(zeros(length(hallwidths),length(perception_weights)));
T.Properties.VariableNames = string(perception_weights);
T.Properties.RowNames = string(hallwidths);

for i = 1:length(stats)
    stat = stats(i);
    idx = find(hallwidths==stat.hallwidth);
%     val = 0;
%     if stat.vx_zero_cross > 1
%        val = 1; 
%     end
    val = stat.max_KU;
    T.(string(stat.perception_weight))(idx) = val;
end

h = heatmap(string(perception_weights),string(hallwidths),table2array(T));
h.XLabel = "Perception Weights";
h.YLabel = "Hall Widths";
h.Title = "More than 1 Vx zero crossing";
% h.YDisplayData = string(hallwidths);
% h.XDisplayData = string(perception_weights);

% close all;
% figure(1);
% hold on;
% subplot(2,1,1);
% semilogx([stats.perception_weight],[stats.time2clear],"LineWidth",2);
% ylabel("Time to clear corner (s)");
% subplot(2,1,2);
% semilogx([stats.perception_weight],[stats.max_KU],"LineWidth",2);
% ylabel("Maximum KU area");
% xlabel("Perception Weight");