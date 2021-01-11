function simulation()
    global sim_dt;
    % Load map
%     map = mapA3();
    map = mapA1();
%     map = mapB();
    map.setParams();
    map.show_cornerMPC = false;
    map.show_cornerWP = false;
    map.show_knownunknown = true;
    map.show_heading = false;
    
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
%         p.mpc_step();
        p.mpc_step2(); % 2 corners
        % Using inputs, step in simulation
        p.motion_step();
        map.update_probs(p);
        % Update plots
        map.update_plot(p,sim_dt);
        % Check map's end condition
        map.check_flag(p);
    end
    
    map.close();
    
    figure(3);
    map.postPlots(p);
end