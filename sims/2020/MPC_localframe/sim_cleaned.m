function sim_cleaned()
    
%     map = map_1corner();
%     map = map_halfcircle();
    map = map_intersection();
%     robot = jackal(map);
    robot = example_UAV(map);
    
    map.plts_enable.constraints = true; % Visualization of constraints
    map.plts_enable.freespace = false; % Blue line that shows where lidar detects free space
    map.plts_enable.lidar = false;
    
    robot.process.noise.enable = true;
    robot.perception.KU.enable = false;
    
    map.init_plot(robot);
    map.init_aux(robot);
        
    params.weights.x = 25;
    params.weights.y = 25;
    params.weights.perc = 0.001;
    params.weights.vx = 50;
    params.weights.vy = 10;
    params.weights.cmd_x = 25;
    params.weights.cmd_y = 25;
    params.max_vel = 2.0;
    params.corner_offset = 1;
    
    while ~map.end_flag
        robot.logic_step(map,params);
        map.update_plot(robot);
        map.update_aux(robot);
        map.update_occ_map(robot);
        map.end_flag_check(robot);
    end
    
    map.close_map();
    
end