% Load data, make array
load("stats.mat");
learning_data = table2array(datatable);

%% Process Data
% Normalize time2clear and maxKU to range [0,1] with mean 0 for each state to be
% trained on
% Create map to access all data points specific to that state value
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
for i = 1:length(keys_cell)
    data = state2data(keys_cell{i});
    s = summary(data(:,["time2clear","max_KU"]));
    mu_t2c = mean(data.time2clear);
    sigma_t2c = std(data.time2clear);
    mu_maxKU = mean(data.max_KU);
    sigma_maxKU = std(data.max_KU);
    for j = 1:size(data,1)
        data.time2clear(j) = (data.time2clear(j)-mu_t2c)/sigma_t2c;
        data.max_KU(j) = (data.max_KU(j)-mu_maxKU)/sigma_maxKU;
    end
    state2data(keys_cell{i}) = data;
end
%% Get training samples
% Define range of reward coefficents to sample from
nn_data = containers.Map();
range_c_time2clear = [0,1];
range_c_maxKU = [0,1];
c_vx_crossing = -50;
c_stuck = -100;
samples_per_state = 100;
for i = 1:length(keys_cell)
    % For each state
    data = state2data(keys_cell{i});
    nn_data(keys_cell{i}) = [];
    for j = 1:samples_per_state
        % For each sample
        c_t2c = range_c_time2clear(1) + rand()*(range_c_time2clear(2)-range_c_time2clear(1));
        c_maxKU = range_c_maxKU(1) + rand()*(range_c_maxKU(2)-range_c_maxKU(1));
        max_cost = c_stuck;
        weights = [];
        % Iterate each weight combination and find the one that yeilds the
        % highest reward
        for k = length(data.time2clear)
            datum = data(k,:);
            cost = c_t2c*datum.time2clear + ...
                c_maxKU*datum.max_KU + ...
                c_vx_crossing*max(datum.vx_zero_cross-1,0) + ...
                c_stuck*datum.stuck;
            if cost > max_cost
                max_cost = cost;
                weights = [datum.x_weight;...
                    datum.y_weight;...
                    datum.perception_weight;...
                    datum.vx_weight;...
                    datum.vy_weight;...
                    datum.cmdx_weight;...
                    datum.cmdy_weight;...
                    datum.corner_offset];
            end
        end
        nn_datum.cost_weights = [c_t2c;c_maxKU;c_vx_crossing;c_stuck];
        nn_datum.best_MPC_weights = weights;
        nn_data(keys_cell{i}) = [nn_data(keys_cell{i});nn_datum];
    end
end



% learning_data = learning_data(randperm(size(learning_data,1)),:);
% input_data = learning_data(:,1:10);
% output_data = learning_data(:,11:15);

% layers = [featureInputLayer(10);...
%     