function sim_prob_collision()
    test_collisions = false;
    map = mapA2();
    map.setParams();
    map.show_cornerMPC = false;
    map.show_cornerWP = false;
    map.show_knownunknown = false;
    map.show_patches = false;
    
    robot = jackal();
    robot.setParams(map);
    
    p_prob = person_prob();
    p_prob.setParams(map);
    
    % Display Map and Robot
    if ~test_collisions
        figure(1);
        % Display person
        p_prob.init_plot(map);
        
        map.initial_plot(robot);
        
        % Display auxilary plot
        figure(2);
        map.initial_aux(robot);
        
        figure(1);
    end
    
    if test_collisions
    trials = 10;
    collisions = 0;
    w = waitbar(0,"Performing Simulations");
    else
       trials = 1; 
    end
    
    for trial = 1:trials
        map.setParams();
        p_prob.setParams(map);
        robot.setParams(map);
    
        while map.end_flag
            % Get corners for robot
            robot.getcorner_MPC(map);
            robot.getcorner_WP(map);
            % Get waypoint for robot
            robot.get_wypt(map);
            % Compute MPC commanded inputs
            %         robot.mpc_step2();
            %         robot.test_step();
            %         robot.mpc_stepDD(); % 2 corners
            robot.mpc_stepvel();
            % Using inputs, step in simulation
            %         robot.motion_stepDD();
            robot.motion_step();
            p_prob.move(robot.dt);
            
            p_prob.update_radprobs(robot);
            if ~test_collisions
                % Update plots
                map.update_plot(robot);
                p_prob.update_plot();
            end
            % Check map's end condition
            map.check_flag(robot);
            if sum(p_prob.mask) == p_prob.patches.num
               map.end_flag = false; 
            end
            %         if robot.t > 6
            %            break
            %         end
%             p.check_collision(robot); *****
        end
        
        if test_collisions
%             if p.collide *******
%                 collisions = collisions + 1;
%             end
            
            waitbar(double(trial)/trials);
        end
    end
    if test_collisions
        close(w);
    end
    map.close();
    debug_data = [];
    if robot.debug.is_true
       debug_data = ["xr","vxs","vys","x_accs","x_jerks","vx_cmds","vy_cmd","v_cmd","e_theta","w_cmd"];
       debug_data = [debug_data; ...
           robot.debug.xrs, ...
           robot.debug.vxs, ...
           robot.debug.vys, ...
           robot.debug.x_accs, ...
           robot.debug.x_jerks, ...
           robot.debug.vx_cmds, ...
           robot.debug.vy_cmds, ...
           robot.debug.v_cmds, ...
           robot.debug.e_thetas, ...
           robot.debug.w_cmds];
       assignin("base","debug_data",debug_data);
    end
    
    if ~test_collisions
        figure(3);
        map.postPlots(robot);
    end
    
    % Correlation between LOS and KU area
    save("robot.mat","robot");
    save("LOS_probs.mat","p_prob");
end