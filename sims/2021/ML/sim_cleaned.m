function sim_cleaned()
    
    map = map_1corner();
%     map = map_halfcircle();
%     map = map_intersection();
%     map = map_equalsign();
%     map = map_triangles();
%     robot = jackal(map);
    robot = example_UAV(map);
    
    map.plts_enable.constraints = false; % Visualization of constraints
    map.plts_enable.freespace = false; % Blue line that shows where lidar detects free space
    map.plts_enable.visible_area = true;
    map.plts_enable.lidar = false; % Visualization of lidar points
    
    robot.process.noise.enable = true;
    robot.perception.KU.enable = true;
    
    map.init_plot(robot);
    map.init_aux(robot);
        
    params.weights.x = 1000;
    params.weights.y = 1000;
    params.weights.perc = 1;
    params.weights.vx = 10;
    params.weights.vy = 10;
    params.weights.cmd_x = 10;
    params.weights.cmd_y = 10;
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