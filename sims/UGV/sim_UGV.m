function sim_UGV()
    %% Initialize objects
%     map = mapA();
%     map = mapA1();

    global sim_dt;
    
    map = mapA1();
    map.setParams();
    map.show_cornerMPC = false;
    map.show_cornerWP = false;
    map.show_knownunknown = true;
    map.show_patches = false;
    
    robot = jackal();
    robot.setParams(map);
    
    % Display Map and Robot
    figure(1);
    map.initial_plot(robot);
    
    % Display auxilary plot
    figure(2);
    map.initial_aux(robot);
    
    figure(1);
    
    while map.end_flag
        % Get corners for robot
        robot.getcorner_MPC(map);
        robot.getcorner_WP(map);
        % Get waypoint for robot
        robot.get_wypt(map);
        % Scale waypoint for safety constraint
%         robot.scale_wypt(map);
        % Compute MPC commanded inputs
%         robot.mpc_step2();
        robot.mpc_acc2vel();
%         robot.test_step();
%         robot.mpc_stepDD(); % 2 corners
%         robot.mpc_stepvel();
        % Using inputs, step in simulation
%         robot.motion_stepDD();
        robot.motion_step(sim_dt);
        map.update_probs(robot);
        % Update plots
        map.update_plot(robot,sim_dt);
        % Check map's end condition
        map.check_flag(robot);
%         if robot.t > 6
%            break 
%         end
    end
    
    map.close();
    debug_data = [];
%     if robot.debug.is_true
%        debug_data = ["xr","vxs","vys","x_accs","x_jerks","vx_cmds","vy_cmd","v_cmd","e_theta","w_cmd"];
%        debug_data = [debug_data; ...
%            robot.debug.xrs, ...
%            robot.debug.vxs, ...
%            robot.debug.vys, ...
%            robot.debug.x_accs, ...
%            robot.debug.x_jerks, ...
%            robot.debug.vx_cmds, ...
%            robot.debug.vy_cmds, ...
%            robot.debug.v_cmds, ...
%            robot.debug.e_thetas, ...
%            robot.debug.w_cmds];
%        assignin("base","debug_data",debug_data);
%     end
    
    figure(3);
    map.postPlots(robot);
    
    % Correlation between LOS and KU area
    save("robot.mat","robot","map");
    assignin("base","robot",robot);
    assignin("base","map",map);
end