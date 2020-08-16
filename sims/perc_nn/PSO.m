% Particle swarm optimization
function x_opt = PSO(x0,cost)
    global feas;
    global NC;
    % cost = cost function handle
    % feas = feasible region
    % dim = number of control variables
    np = 1000; % number of particles
    ni = 1000; % number of iterations
    omega = 1;
    phi_p = 1;
    phi_g = 1;
    epsilon = 0.05;
    % Initilialize particles
    d = size(feas,1)*NC;
    feas_lower = repmat(feas(:,1),NC,np);
    feas_upper = repmat(feas(:,2),NC,np);

    lower = feas(:,1);
    lower = repmat(lower,NC,np);
    interval = feas(:,2)-feas(:,1);
    interval = repmat(interval,NC,np);
    ps = rand(d,np);
    ps = interval.*ps + lower; % Randomly initialize particles
    best_ps = ps; % assign best indv position of particle to current position
    best_p_cost = cost(x0,ps); % Track best cost of each particle
    costs = cost(x0,ps);
    min_cost = min(costs);
    best_g_cost = min_cost;
    best_g = ps(:,costs==min_cost);
    best_gs = repmat(best_g,1,np);
    vs = (2*rand(d,np) - ones(d,np)).*interval; % Initial velocities of particles
    
    count = 0;
    while norm(mean(best_ps,2)-best_g,1) > epsilon && count < ni
        rp = rand(d,np); % random # for particle's best component
        rg = rand(d,np); % random # for global best component
        vs = omega*vs + phi_p*rp.*(best_ps - ps) + phi_g*rg.*(best_gs - ps); % vel for each particle
        ps = ps + vs; % update particle position
        ps = max(min(ps,feas_upper),feas_lower); % Cut off at max/min for each
        
        new_costs = cost(x0,ps); % Find cost for each particle
        min_cost = min(new_costs); % Best cost of this iter
        
        best_g_new = ps(:,new_costs==min_cost); % particle of current best cost
        best_g_new = best_g_new(:,randi(size(best_g_new,2))); % If more than one particle with same cost, choose random
        if min_cost < best_g_cost % new cost is lower than lowest recorded
            best_g_cost = min_cost; % new best cost
            best_g = best_g_new; % particle of new lowest (best) cost
            best_gs = repmat(best_g,1,np); % particle of best cost repeated
        end
        
        best_p_costs = min(new_costs,costs); % Best individual costs
        index_bestnew = (new_costs==best_p_costs); % index of new costs if better
        index_bestold = (costs==best_p_costs); % index of old costs if better
        best_new = ps; % make copy of current particle positions
        best_old = best_ps; % make copy of old personal bests
        best_new(:,~index_bestnew)=0; % ps whose new position has a better cost
        best_old(:,~index_bestold)=0; % ps whose old position has a better cost
        best_ps = best_new + best_old; % best individual positions
        
        costs = new_costs;
        count = count + 1;
%         disp(count);
%         fprintf("Best position: %f, %f\n",best_g(1),best_g(2));
    end
    
%     disp(count);
%     fprintf("Best position: %f, %f\n",best_g(1),best_g(2));
    x_opt = best_g;
    
end                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             