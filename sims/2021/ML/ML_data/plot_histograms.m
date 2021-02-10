close all;

% Load data, make array
load("stats.mat");
learning_data = table2array(datatable);

% Script to plot the histograms of the unscaled and scaled data
% This is mostly for data analysis
state2data = containers.Map();
for i = 1:size(learning_data,1)
    learning_datum = learning_data(i,:);
    current_hallwidth = learning_datum(1);
    upcoming_hallwidth = learning_datum(2);
    max_sensing_range= learning_datum(3);
    state_char = sprintf('%d,%d,%d',current_hallwidth,upcoming_hallwidth,max_sensing_range);
    if state2data.Count==0
        % Initializie values inside map
        state2data(state_char) = datatable(i,:);
    end
    if isKey(state2data,state_char)
        samestate_datatable = state2data(state_char);
        state2data(state_char) = [samestate_datatable;datatable(i,:)];
    else
        state2data(state_char) = datatable(i,:);
    end
end
% For each state value, normalize time2clear and maxKU variables
keys_cell = keys(state2data);
% For each state, plot histogram of time2clear and maxKU
for i = 1:length(keys_cell)
    key = keys_cell{i};
    data = state2data(key);
    figure(i);
    subplot(1,2,1);
    h_ku = histogram(data.max_KU);
    xlabel("KU area (s)");
    subplot(1,2,2);
    h_t2c = histogram(data.time2clear);
    xlabel("Time to clear (s)");
    sgtitle(sprintf("State: %s",key));
end
