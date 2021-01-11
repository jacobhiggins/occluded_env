function sim_cleaned()
    
    map = map_1corner();
%     robot = jackal(map);
    robot = example_UAV(map);
    
    map.init_plot(robot);
    map.init_aux(robot);
        
    params.weights.x = 50;
    params.weights.y = 50;
%     weights.perc = 1000;
    params.weights.perc = 0.01;
    params.weights.cmd_x = 25;
    params.weights.cmd_y = 25;
    params.max_vel = 2.0;
    params.safe = true;
    
    while ~map.end_flag
        robot.logic_step(map,params);
        map.update_plot(robot);
        map.update_aux(robot);
        map.end_flag_check(robot);
    end
    
    map.close_map();
    
end