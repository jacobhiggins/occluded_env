function sim_ML()

    % ===== Position Weight ===== %
    position_weights = [1,10,50,100,1000];
%     position_weights = [50];
%     position_weights = [0.01,1000];
    % ===== Velocity Weight ===== %
    params.weights.vx = 10;
    params.weights.vy = 10;
    % ===== Command Weight ===== %
    params.weights.cmd_x = 25;
    params.weights.cmd_y = 25;
    % ===== Perception Weight ===== %
    perception_weights = [1,10,50,100,1000];
%     perception_weights = [0.01,1000];
%     perception_weights = 100;
    % ===== Corner Offset ===== %
    corner_offsets = [0,1];
    % ===== Current Hallwidth ===== &
    current_hallwidths = [1,5,10,15,20,25];
%     current_hallwidths = [1,10];
%     current_hallwidths = [5];
    % ===== Upcoming Hallwidth ===== %
    upcoming_hallwidths = [1,5,10,15,20,25];
%     upcoming_hallwidths = [5];
    % ===== Sensing Range ===== %
    max_sensing_ranges = [10];
    
    i = 0;
    stats = [];
    
    wbar = waitbar(0,'Gathering Data');
    for x_weight = position_weights
        params.weights.x = x_weight;
        for y_weight = position_weights
            params.weights.y = y_weight;
            for w = current_hallwidths
                halldims.w = [w,5];
                halldims.l = [10,w+1];
                map = map_1corner(halldims);
                map.vid.rec = false;
                for perception_weight = perception_weights
                    for max_sensing_range = max_sensing_ranges
                        for corner_offset = corner_offsets
                            params.corner_offset = corner_offset;
                            % Create stats that holds state + actions
                            stat.current_hallwidth = w;
                            stat.upcoming_hallwidth = 5;
                            stat.max_sensing_range = max_sensing_range;
                            stat.perception_weight = perception_weight;
                            stat.x_weight = x_weight;
                            stat.y_weight = y_weight;
                            stat.vx_weight = params.weights.vx;
                            stat.vy_weight = params.weights.vy;
                            stat.cmdx_weight = params.weights.cmd_x;
                            stat.cmdy_weight = params.weights.cmd_y;
                            stat.corner_offset = corner_offset;
                            disp(stat); % Display state to terminal
                            % Run Simulation
                            map.end_flag = false;
                            robot = example_UAV(map);
                            robot.lidar.maxRange = max_sensing_range;
                            params.weights.perc = perception_weight;
%                             map.init_plot(robot); % Uncomment for see plot
                            while ~map.end_flag
                                robot.logic_step(map,params);
%                                 map.update_plot(robot); % Uncomment to update plot
                                map.end_flag_check(robot);
                                drawnow();
                            end
                            % Save stats
                            
                            stat.time2clear = robot.time;
                            stat.max_KU = robot.dc.stats.max_KU;
                            stat.vx_zero_cross = robot.dc.stats.vx_zero_cross;
                            stat.stuck = robot.dc.stats.stuck;
                            stats = [stats stat];
                            % Save table
                            if i==0
                                datatable = struct2table(stat);
                            else
                                datatable = [datatable;struct2table(stat)];
                            end
                            i = i + 1;
                            waitbar(i/(length(perception_weights)*length(current_hallwidths)*length(corner_offsets)*length(position_weights)^2),wbar,'Gathering Data');
                            clear robot
                            map.close_map();
                        end
                    end
                end
            end
        end
    end
    close(wbar);
    save("./ML_data/stats.mat","stats","datatable");
    
end