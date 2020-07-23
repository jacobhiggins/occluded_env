function sim_UGV()
    %% Initialize objects
    map = mapA();
    map.setParams();
    
    robot = UGV();
    robot.setParams(map);
    
    % Display Map and Robot
    figure(1);
    map.initial_plot(robot);
    
    % Display auxilary plot
    figure(2);
    map.initial_aux(robot);
    
    while map.end_flag
        % Get corners for robot
        robot.getcorner_MPC(map);
        robot.getcorner_WP(map);
        % Get waypoint for robot
        robot.get_wypt(map);
        % Compute MPC commanded inputs
        %         p.mpc_step();
        robot.mpc_step2(); % 2 corners
        % Using inputs, step in simulation
        robot.motion_step();
        % Update plots
        map.update_plot(robot);
        % Check map's end condition
        map.check_flag(robot);
    end
    
    map.close();
    
    figure(3);
    map.postPlots(robot);
    
end