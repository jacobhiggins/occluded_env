function sim_ML()

    perception_weights = [0.01,0.1,1,10,100,1000];
%     perception_weights = [0.01,1000];
    hallwidths = [1,3,5,7,9];
    position_weights = [0.01,0.1,1,10,100,1000];
%     hallwidths = [9];
    corner_offsets = [0];
%     params.weights.x = 50;
%     params.weights.y = 50;
    params.weights.cmd_x = 25;
    params.weights.cmd_y = 25;
    
    i = 0;
    stats = [];
    wbar = waitbar(0,'Gathering Data');
    for corner_offset = corner_offsets
        for x_weight = position_weights
            params.weights.x = x_weight;
            for y_weight = position_weights
                params.weights.y = y_weight;
                for w = hallwidths
                    halldims.w = [w,5];
                    halldims.l = [10,10];
                    map = map_1corner(halldims);
                    map.vid.rec = false;
                    params.corner_offset = corner_offset;
                    for perception_weight = perception_weights
                        map.end_flag = false;
                        robot = example_UAV(map);
                        params.weights.perc = perception_weight;
%                         map.init_plot(robot);
                        while ~map.end_flag
                            robot.logic_step(map,params);
%                             map.update_plot(robot);
                            map.end_flag_check(robot);
                            drawnow();
                        end
                        stat.hallwidth = w;
                        stat.time2clear = robot.dc.stats.time2clear;
                        stat.max_KU = robot.dc.stats.max_KU;
                        stat.perception_weight = perception_weight;
                        stat.x_weight = x_weight;
                        stat.y_weight = y_weight;
                        stat.vx_zero_cross = robot.dc.stats.vx_zero_cross;
                        stat.stuck = robot.dc.stats.stuck;
                        stat.corner_offset = corner_offset;
                        stats = [stats stat];
                        i = i + 1;
                        waitbar(i/(length(perception_weights)*length(hallwidths)*length(corner_offsets)*length(position_weights)^2),wbar,'Gathering Data');
                        clear robot
                        map.close_map();
                    end
                end
            end
        end
    end
    close(wbar);
    save("./ML_data/stats.mat","stats");
    
end