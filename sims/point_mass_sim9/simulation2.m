function simulation2(record)
    % Load map
    map = mapA();
%     map = mapB();
    map.setParams();
    
    % Initialize point
%     p = point();
    p = point_2();
%     p.maxRad = 20;
%     p.r = p.maxRad;
    p.initial_params(map);
    
    % Display Map and Robot
    figure(1);
    map.initial_plot(p);
    
    % Display auxilary plot
    figure(2);
    map.initial_aux(p);
    
    while map.end_flag
        % Get corners for robot
        p.getcorner_MPC(map);
        p.getcorner_WP(map);
        % Get waypoint for robot
        p.get_wypt(map);
        % Compute MPC commanded inputs
        p.mpc_step();
        % Using inputs, step in simulation
        p.motion_step();
        % Update plots
        map.update_plot(p);
        % Check map's end condition
        map.check_flag(p);
    end
    
    map.close();
    
    figure(3);
    map.postPlots(p);
end