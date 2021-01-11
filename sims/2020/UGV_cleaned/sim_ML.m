function sim_ML()

    map = map_1corner();
    
%     perception_weights = 1:50:1000;
    perception_weights = [0.01,0.1,1,10,100,1000];
    weights.x = 10;
    weights.y = 50;
    weights.cmd_x = 25;
    weights.cmd_y = 25;
    i = 0;
    
%     try
%        load("./ML_data/stats.mat"); 
%     catch
%        stats = [];
%     end
    stats = [];
    
    w = waitbar(0,'Gathering Data');
    for perception_weight = perception_weights
        map.end_flag = false;
        robot = example_UAV(map);
        weights.perc = perception_weight;
%         map.init_plot(robot);
        while ~map.end_flag
            robot.logic_step(map,weights);
%             map.update_plot(robot);
            map.end_flag_check(robot);
            drawnow();
        end
        stat.time2clear = robot.dc.stats.time2clear;
        stat.max_KU = robot.dc.stats.max_KU;
        stat.perception_weight = perception_weight;
        stats = [stats stat];
        i = i + 1;
        waitbar(i/length(perception_weights),w,'Gathering Data');
        clear robot
        map.close_map();
    end
    close(w);
    

    
    save("./ML_data/stats.mat","stats");
    
end