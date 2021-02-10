% clear all;
close all;
load("stats.mat");

%% Plot cost vs (peceptionweight/xweight and currenthallwidth)
close all;
current_hallwidths = table2array(unique(datatable(:,"current_hallwidth")));
perception_weights = table2array(unique(datatable(:,"perception_weight")));
x_weights = table2array(unique(datatable(:,"x_weight")));
y_weights = table2array(unique(datatable(:,"y_weight")));
% T = array2table(-100*ones(length(current_hallwidths),length(perception_weights)));
% T.Properties.VariableNames = string(perception_weights);
% T.Properties.RowNames = string(perception_weights);
xdata = []; % hallwidth
ydata = []; % perception weights/y weight
zdata = []; % cost
cdata = [];
cmap = cool(length(y_weights));
for i = 1:size(datatable,1)
    row = table2array(datatable(i,:));
    current_hallwidth = row(1);
    perception_weight = row(4);
    x_weight = row(5);
    y_weight = row(6);
    corner_offset = row(11);
    if abs(corner_offset-1)<0.0001
        continue;
    end
    time2clear = row(12);
    maxKU = row(13);
    vx_zero = row(14);
    stuck = row(15);
    cost_val = cost(time2clear,maxKU,vx_zero,stuck);
    xdata(end+1) = current_hallwidth;
    ydata(end+1) = log10(perception_weight/x_weight);
    zdata(end+1) = cost_val;
    cdata(end+1,1:3) = cmap(find(y_weight==y_weights),:);
end

figure(1);
plt = scatter3(xdata,ydata,zdata,50*ones(size(xdata)),cdata);
xlabel("current hall width");
ylabel("perception weight / x weight");
zlabel("reward");
set(gca, 'ZScale', 'log')
colormap(cmap);
cb = colorbar;
cb.Ticks = log10(y_weights);
cb.Limits = [min(log10(y_weights)) max(log10(y_weights))];
drawnow;
% %%
% % Plot when vx crosses zero twice
% hallwidths = unique([stats.hallwidth]);
% perception_weights = unique([stats.perception_weight]);
% T = array2table(zeros(length(hallwidths),length(perception_weights)));
% T.Properties.VariableNames = string(perception_weights);
% T.Properties.RowNames = string(hallwidths);
% 
% for i = 1:length(stats)
%     stat = stats(i);
%     idx = find(hallwidths==stat.hallwidth);
% %     val = 0;
% %     if stat.vx_zero_cross > 1
% %        val = 1; 
% %     end
%     val = stat.max_KU;
%     T.(string(stat.perception_weight))(idx) = val;
% end
% 
% h = heatmap(string(perception_weights),string(hallwidths),table2array(T));
% h.XLabel = "Perception Weights";
% h.YLabel = "Hall Widths";
% h.Title = "More than 1 Vx zero crossing";
% % h.YDisplayData = string(hallwidths);
% % h.XDisplayData = string(perception_weights);
% 
% % close all;
% % figure(1);
% % hold on;
% % subplot(2,1,1);
% % semilogx([stats.perception_weight],[stats.time2clear],"LineWidth",2);
% % ylabel("Time to clear corner (s)");
% % subplot(2,1,2);
% % semilogx([stats.perception_weight],[stats.max_KU],"LineWidth",2);
% % ylabel("Maximum KU area");
% % xlabel("Perception Weight");