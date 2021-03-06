classdef point_2 < handle
   properties
      x
      y
      xc_mpc_r
      yc_mpc_r
      xc_mpc_l
      yc_mpc_l
      rc_active = false;
      lc_active = false;
      xc_wp
      yc_wp
      last_sec
      wypt = struc('x',[],'y',[]);
      M_mpc
      flip
      vx
      vy
      ax
      ay
      v_max
      a_max
      dt
      t
      r
      rs = [0.0];
      maxRad
      trail = struct('x',[],'y',[]); % Motion trail
      proj_mot = struct('x',[],'y',[]); % Projected motion
      cmd_input = struct('x',[0.0],'y',[0.0],'name',"Acceleration");
      cmd_inputs = struct('x',[0.0],'y',[0.0]);
      vs = struct('vxs',[0.0],'vys',[0.0]);
      LOSs = [];
      ts = [0.0];
      LOS = [0.0];
      current_sec = 1; % current section that robot is in
      current_owall = -100; % current outer wall distance from corner
      next_owall = -100; % next hallway wall distance
      % MPC vars
      N = 50; % Control Horizon
      Nu = 3; % Decision variables
      MPCinput = struct('x0',[],'x',[],'y',[],'yN',[],'W',[],'WN',[]);
      MPCoutput = struc('x',[],'u',[]);
      large_num = 100000;
      ku = struct("areas",[0.0]);
      safe = false; % Safe to move around corners
      stop_dist = 0.5; % m
   end
   methods
       function initial_params(obj,map)
           obj.dt = 0.1;
           obj.t = 0.0;
           obj.x = map.pose_start.x;
           obj.y = map.pose_start.y;
           obj.vx = 0;
           obj.vy = 0;
           obj.ax = 0;
           obj.ay = 0;
           obj.xc_mpc_r = map.corners_r(1,1);
           obj.yc_mpc_r = map.corners_r(1,2);
           obj.xc_mpc_l = -1*obj.large_num;
           obj.yc_mpc_l = obj.yc_mpc_r;
           obj.xc_wp = map.corners_r(1,1);
           obj.yc_wp = map.corners_r(1,2);
           obj.last_sec = false;
           obj.wypt.x = obj.x;
           obj.wypt.y = obj.y + obj.maxRad;
           obj.MPCinput.u = zeros(obj.N,obj.Nu);
           obj.trail.x = obj.x;
           obj.trail.y = obj.y;
           obj.proj_mot.x = obj.x*ones(1,obj.N);
           obj.proj_mot.y = obj.y*ones(1,obj.N);
           obj.maxRad = map.maxRad_suggest;
           obj.r = obj.maxRad;
           obj.getcorner_MPC(map);
       end
       function getcorner_MPC(obj,map)
          persistent curr_rc;
          persistent curr_lc;
          if isempty(curr_rc) || obj.t < 0.001
              curr_rc = 1;
              curr_lc = 1;
          end
          M = map.Ms_mpc{curr_rc};
          [~,dely_rc] = c2u(obj.x,obj.y,obj.xc_mpc_r,obj.yc_mpc_r,M);
          [~,dely_lc] = c2u(obj.x,obj.y,obj.xc_mpc_l,obj.yc_mpc_l,M);
          if dely_rc > 0
              curr_rc = min(curr_rc + 1,size(map.corners_r,1));
              obj.rc_active = false;
              obj.safe = false;
          end
          
          
          obj.xc_mpc_r = map.corners_r(curr_rc,1);
          obj.yc_mpc_r = map.corners_r(curr_rc,2);
          obj.xc_mpc_l = map.corners_l(curr_lc,1);
          obj.yc_mpc_l = map.corners_l(curr_lc,2);
          
          corner_r.x = obj.xc_mpc_r;
          corner_r.y = obj.yc_mpc_r;
          
          corner_l.x = obj.xc_mpc_l;
          corner_l.y = obj.yc_mpc_l;
          
          if ~map.isVisible(corner_r,obj)
              obj.rc_active = false;
          else
              obj.rc_active = true;
          end
          if dely_lc>0 && map.isVisible(corner_l,obj)
              curr_lc =  min(curr_lc + 1,size(map.corners_l,1));
              obj.lc_active = false;
          elseif map.isVisible(corner_l,obj)%dely_lc > -obj.maxRad
              obj.lc_active = true;
          end
          obj.M_mpc = map.Ms_mpc{curr_rc};
          obj.current_owall = map.walls_mpc(curr_rc);
          obj.next_owall = map.walls_mpc(min(curr_rc+1,length(map.walls_mpc)));
       end
       function getcorner_WP(obj,map)
           persistent curr_c; % current corner index
           persistent next_c;
           if isempty(curr_c)
               curr_c = 1;
               next_c = 2;
           end
           curr_c = min(curr_c,size(map.corners_r,1));
           next_c = min(next_c,size(map.corners_r,1));
           xc1 = map.corners_r(curr_c,1);
           yc1 = map.corners_r(curr_c,2);
           xc2 = map.corners_r(next_c,1);
           yc2 = map.corners_r(next_c,2);
           avoid = map.corners_r(curr_c,3);
           phi1 = atan2(yc1-obj.y,xc1-obj.x);
           phi2 = atan2(yc2-obj.y,xc2-obj.x);
           if sign(phi2 - phi1) == avoid
               curr_c = curr_c + 1;
               next_c = next_c + 1;
           end
           obj.xc_wp = map.corners_r(curr_c,1);
           obj.yc_wp = map.corners_r(curr_c,2);
%            M = Ms{curr_c};
%            flip = corners_r(curr_c,3);
           obj.current_sec = curr_c;
           if obj.current_sec == size(map.corners_r,1)
               obj.last_sec = true;
           end

       end
       function get_wypt(obj,map)
           persistent dists; % Distances between waypoints
           persistent vecs; % vectors between waypoints
           persistent c_base; % current waypoint base
           if isempty(dists)
               for i = 1:length(map.wypt_bases)-1
                   wypta = map.wypt_bases(i,:);
                   wyptb = map.wypt_bases(i+1,:);
                   vec = wyptb - wypta;
                   vec = vec/norm(vec,2);
                   dists = [dists;norm(wypta-wyptb,2)];
                   vecs = [vecs;vec];
                   c_base = 1;
               end
           end
           c_base = min(length(dists),c_base); % current waypoint index for circle
           wpb = map.wypt_bases(c_base,:); % current waypoint index for radius
           next_wpb = map.wypt_bases(min(c_base+1,size(map.wypt_bases,1)),:); % next waypoint for circle
           next_next_wpb = map.wypt_bases(min(c_base+2,size(map.wypt_bases,1)),:); % next next waypoint for circle
           vec = vecs(c_base,:); % vector between current and next waypoint for circle
           m = vec(2)/vec(1); % slope of line bewteen current and next waypoint for circle
           
           % Determination of radius using appropriate waypoints
           n = length(map.wypt_bases);
           xm2 = map.wypt_bases(min(obj.current_sec+1,n),1); % ALSO CHANGED
           ym2 = map.wypt_bases(min(obj.current_sec+1,n),2);
           xm3 = map.wypt_bases(min(obj.current_sec+2,n),1);
           ym3 = map.wypt_bases(min(obj.current_sec+2,n),2);
           theta2 = atan2(ym3-ym2,xm3-xm2);
           
           % Set radius using appropriate waypoints
           obj.set_radius(xm2,ym2,theta2,1);
           
           [xis,yis] = linecirc(m,0,obj.x-wpb(1),obj.y-wpb(2),obj.r); % x's and y's where circle intersects line
           tmp1 = -Inf;
           % Pick point of intersection that has the highest dot product with vec
           for i = 1:2
               tmp = vec(1)*xis(i) + vec(2)*yis(i);
               if tmp > tmp1
                   xi = xis(i);
                   yi = yis(i);
                   tmp1 = tmp;
               end
           end
           try
               waypoint.x = xi + wpb(1);
               waypoint.y = yi + wpb(2);
               vec1 = [waypoint.x-obj.wypt.x;waypoint.y-obj.wypt.y];
           catch
               vec1 = -vec';
           end
           if vec*vec1 < 0
               waypoint = obj.wypt;
           end
           obj.wypt = waypoint;
           if norm([obj.x-next_wpb(1),obj.y-next_wpb(2)],2) < obj.r
               c_base = c_base + 1;
           end
%            obj.scale_wypt(map); % Place at corner if not safe
           obj.shift_wypt(map); % Test points on model traj until safe
       end
       function scale_wypt(obj,map)
           % Check expected dist to collision for safety
           % If not safe, project waypoint onto corner line
           num_points = 25;
           del_x = abs(obj.x - obj.wypt.x);
           del_y = abs(obj.y - obj.wypt.y);
           xs_probe = obj.x:del_x/num_points:obj.wypt.x;
           ys_probe = obj.y:del_y/num_points:obj.wypt.y;
           x_avg = 0.0;
           y_avg = 0.0;
           prob_product = 1;
           for i = 1:num_points
               % Get probe x-y position
               if isempty(xs_probe)
                   x_probe = obj.x;
               else
                   x_probe = xs_probe(i);
               end
               if isempty(ys_probe)
                   y_probe = obj.y;
               else
                   y_probe = ys_probe(i);
               end
               % Get index of probability that is closest to that position
               shifted_centers = abs(map.patches.centers - [x_probe;y_probe]);
               is = logical(shifted_centers(1,:) <= map.patches.width/2).*logical(shifted_centers(2,:) <= map.hws(2)/2);
               prob = map.patches.probs(logical(is));
               if ~isempty(prob)
                   prob = prob(1);
               else
                   prob = 1;
               end
               if i < num_points
                   x_avg = x_avg + x_probe*prob_product*(1-prob);
                   y_avg = y_avg + y_probe*prob_product*(1-prob);
               else
                   x_avg = x_avg + x_probe*prob_product;
                   y_avg = y_avg + y_probe*prob_product;
               end
               prob_product = prob_product*prob;
           end
           % Using dist + theta (instead of exact point) ensures
           % the waypoint is always in the right direction
           
           [wypt_primex,wypt_primey] = c2u(obj.wypt.x,obj.wypt.y,obj.x,obj.y,obj.M_mpc);
           [corner_primex,corner_primey] = c2u(obj.xc_mpc_r,obj.yc_mpc_r,obj.x,obj.y,obj.M_mpc);
           y_cp_prime = corner_primey; % projection of wypt onto corner plane
           x_cp_prime = wypt_primex*corner_primey/wypt_primey;
           [x_cp,y_cp] = u2c(x_cp_prime,y_cp_prime,obj.x,obj.y,obj.M_mpc);
           
           vec_cp_avg = [x_avg-x_cp;y_avg-y_cp]; % vector from cp to wypt
           vec_r_wypt = [obj.wypt.x-obj.x;obj.wypt.y-obj.y]; % vector from r to wypt
           vec_r_wypt = vec_r_wypt/norm(vec_r_wypt);
           
           vec = obj.M_mpc*([x_avg;y_avg]-[obj.x;obj.y]);
           dist_to_exp = norm([x_avg;y_avg]-[obj.x;obj.y]);
           dist_to_fast_wypt = norm([obj.wypt.x;obj.wypt.y] - [obj.x;obj.y]);
           delta_dist = dist_to_fast_wypt - dist_to_exp;
           theta = atan2(obj.wypt.y - obj.y,obj.wypt.x - obj.x);
           if vec_r_wypt'*vec_cp_avg > obj.stop_dist
              obj.safe = true; 
           end
           if ~obj.safe
               if wypt_primey > y_cp_prime
                  obj.wypt.x = x_cp;
                  obj.wypt.y = y_cp;
               end
           end
%            obj.wypt.x = obj.x + dist*cos(theta); % Old way, place
%            obj.wypt.y = obj.y + dist*sin(theta); % wypt at expectation
       end
       function shift_wypt(obj,map) % NEW 09/23/2020
           persistent I_old; % index of last waypoint
           if isempty(I_old)
              I_old = 0; 
           end
           % Use expected distance to collision for safety
           % If not safe, follow model traj backwards until you find a safe
           % point. Stop at corner
           dist = obj.expected_dist2coll(map,obj.wypt);
           if dist > obj.stop_dist || dist < 0
              if dist > obj.stop_dist
                  obj.safe = true;
              end
              return;
           end
%            if ~obj.safe
              % Get next closest point on model trajectory
              [~,I] = min(vecnorm([map.model_traj.points(:,1),map.model_traj.points(:,2)]-[obj.wypt.x,obj.wypt.y],2,2));
              test_point.x = map.model_traj.points(I,1);
              test_point.y = map.model_traj.points(I,2);
              tp_primey = 100; % Initialize to enter while loop
              while tp_primey > 0.01 && I > I_old
                  dist = obj.expected_dist2coll(map,test_point);
                  % If this point satisfies the condition, break from
                  % while loop
                  if dist > obj.stop_dist || dist < 0
                     if dist > obj.stop_dist
                        obj.safe = true; 
                     end
                     break;
                  end
                  I = I - 1;
                  test_point.x = map.model_traj.points(I,1);
                  test_point.y = map.model_traj.points(I,2);
                  [~,tp_primey] = c2u(test_point.x,test_point.y,obj.x,obj.y,obj.M_mpc);
              end
              I_old = I; % Make sure new wypt is farther than last wypt
              obj.wypt.x = test_point.x;
              obj.wypt.y = test_point.y;
           end
%        end
       function dist = expected_dist2coll(obj,map,test_point)
           num_points = 25;
           del_x = abs(obj.x - test_point.x);
           del_y = abs(obj.y - test_point.y);
           xs_probe = obj.x:del_x/num_points:test_point.x;
           ys_probe = obj.y:del_y/num_points:test_point.y;
           x_avg = 0.0;
           y_avg = 0.0;
           prob_product = 1;
           for i = 1:num_points
               % Get probe x-y position
               if isempty(xs_probe)
                   x_probe = obj.x;
               else
                   x_probe = xs_probe(i);
               end
               if isempty(ys_probe)
                   y_probe = obj.y;
               else
                   y_probe = ys_probe(i);
               end
               % Get index of probability that is closest to that position
               shifted_centers = abs(map.patches.centers - [x_probe;y_probe]);
               is = logical(shifted_centers(1,:) <= map.patches.width/2).*logical(shifted_centers(2,:) <= map.hws(2)/2);
               prob = map.patches.probs(logical(is));
               if ~isempty(prob)
                   prob = prob(1);
               else
                   prob = 1;
               end
               if i < num_points
                   x_avg = x_avg + x_probe*prob_product*(1-prob);
                   y_avg = y_avg + y_probe*prob_product*(1-prob);
               else
                   x_avg = x_avg + x_probe*prob_product;
                   y_avg = y_avg + y_probe*prob_product;
               end
               prob_product = prob_product*prob;
           end
           % Get projection of waypoint on corner plane
           [tp_primex,tp_primey] = c2u(test_point.x,test_point.y,obj.x,obj.y,obj.M_mpc);
           [~,corner_primey] = c2u(obj.xc_mpc_r,obj.yc_mpc_r,obj.x,obj.y,obj.M_mpc);
           y_cp_prime = corner_primey; % projection of wypt onto corner plane
           x_cp_prime = tp_primex*corner_primey/tp_primey;
           [x_cp,y_cp] = u2c(x_cp_prime,y_cp_prime,obj.x,obj.y,obj.M_mpc);
           
           % Get vector between robot and expect point of collision
           vec_cp_avg = [x_avg-x_cp;y_avg-y_cp]; % vector from cp to wypt
           vec_r_tp = [test_point.x-obj.x;test_point.y-obj.y]; % vector from r to wypt
           vec_r_tp = vec_r_tp/norm(vec_r_tp); % Normalize (don't care about this distance)
           
           dist = vec_r_tp'*vec_cp_avg;
       end
       function set_radius(obj,xm2,ym2,theta2,dist_frac)
           r = obj.maxRad;
           if(obj.last_sec)
               obj.r = r;
              return 
           end
           theta = atan2(obj.yc_wp-obj.y,obj.xc_wp-obj.x);
           if theta ~= theta2
               A = [cos(theta),-cos(theta2);...
                   sin(theta),-sin(theta2)];
               b = [xm2-obj.x;ym2-obj.y];
               vec = A\b;
               r = vec(1);
               if (r > obj.r && r < obj.maxRad) || r < 0
                   if r > 0
                       r = min(obj.r + obj.maxRad/50,r);
                   else
                       r = obj.r + obj.maxRad/50;
                   end
               end
               r = min(dist_frac*obj.maxRad,r);
               if r > obj.maxRad
                   r = min(obj.r + obj.maxRad/50,obj.maxRad);
               end
           end
           obj.r = r;
       end
       function mpc_step(obj)
           left_bound = obj.current_owall;
           right_bound = 0;
           if obj.current_sec==3
               right_bound = 15;
           end
           xr_in = obj.x;
           yr_in = obj.y;
           xc_in = obj.xc_mpc_r;
           yc_in = obj.yc_mpc_r;
           xg_in = obj.wypt.x;
           yg_in = obj.wypt.y;
           vx_in = obj.vx;
           vy_in = obj.vy;
           ax_in = obj.ax;
           ay_in = obj.ay;
           dt = obj.dt;
           
           [xr,yr] = c2u(xr_in,yr_in,xc_in,yc_in,obj.M_mpc);
           [xg,yg] = c2u(xg_in,yg_in,xc_in,yc_in,obj.M_mpc);
           [vx,vy] = c2u(vx_in,vy_in,0,0,obj.M_mpc);
           [ax,ay] = c2u(ax_in,ay_in,0,0,obj.M_mpc);
           [xc,yc] = c2u(xc_in,yc_in,xc_in,yc_in,obj.M_mpc);
           
           % Visibility Objective
           m = (yc - yr)/(xc - xr);
           phi = atan(m);
           phi_y = phi/yr;
           
           % Position Constraints
           m_inv = 1/m;
           
           % If projected motion doesn't reach corner, don't set slope
           % constraint
           try
               x_last = obj.MPCinput.x(end,1); % Last (x,y) coordinate of projected motion
               y_last = obj.MPCinput.x(end,2);
               [~,y_last] = c2u(x_last,y_last,xc_in,yc_in,obj.M_mpc);
               if y_last < 0
                   m_inv = 0;
               end
           catch
           end
           
           var = phi_y;
           var_des = 0;
           safe = 0;
           delta_x = 1;
           
           obj.MPCinput.x0 = [xr,yr,vx,vy,xc,yc,ax,ay,var,safe,left_bound,right_bound,delta_x,m_inv];
           
           obj.MPCinput.x = [xr*ones((obj.N+1),1) ...
               yr*ones((obj.N+1),1) ...
               vx*ones(obj.N+1,1) ...
               vy*ones(obj.N+1,1) ...
               xc*ones((obj.N+1),1) ...
               yc*ones((obj.N+1),1) ...
               ax*ones(obj.N+1,1) ...
               ay*ones(obj.N+1,1) ...
               var*ones((obj.N+1),1) ...
               safe*ones((obj.N+1),1) ...
               left_bound*ones(obj.N+1,1) ...
               right_bound*ones(obj.N+1,1) ...
               delta_x*ones(obj.N+1,1) ...
               m_inv*ones(obj.N+1,1)];
           
           obj.MPCinput.y = [xg*ones(obj.N,1), yg*ones(obj.N,1), var_des*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1)];
           obj.MPCinput.yN = [xg yg var_des 0];
           
           % x, y, perception, safety, ax_dot, ay_dot, epsilon_leftwall, epsilon_safety
           A = diag([500 500 50000000 0.001 250 250 50000000 0.001]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = 1*diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver6( obj.MPCinput );
           as = obj.MPCoutput.u(1,:);
           ax_dot = as(1);
           ay_dot = as(2);
           obj.ax = ax + ax_dot*dt;
           obj.ay = ay + ay_dot*dt;
           [ax,ay] = c2u(obj.ax,obj.ay,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = ax;
           obj.cmd_input.y = ay;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_in*ones(obj.N+1,1) yc_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
       end
       function mpc_step2(obj)
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           if obj.current_sec==3
               right_bound = 15;
           end
           xr_in = obj.x;
           yr_in = obj.y;
           xc_r_in = obj.xc_mpc_r;
           yc_r_in = obj.yc_mpc_r;
           xc_l_in = obj.xc_mpc_l;
           yc_l_in = obj.yc_mpc_l;
           xg_in = obj.wypt.x;
           yg_in = obj.wypt.y;
           vx_in = obj.vx;
           vy_in = obj.vy;
           ax_in = obj.ax;
           ay_in = obj.ay;
           dt = obj.dt;
           
           [xr,yr] = c2u(xr_in,yr_in,xc_r_in,yc_r_in,obj.M_mpc);
           [xg,yg] = c2u(xg_in,yg_in,xc_r_in,yc_r_in,obj.M_mpc);
           [vx,vy] = c2u(vx_in,vy_in,0,0,obj.M_mpc);
           [ax,ay] = c2u(ax_in,ay_in,0,0,obj.M_mpc);
           [xc_r,yc_r] = c2u(xc_r_in,yc_r_in,xc_r_in,yc_r_in,obj.M_mpc);
           [xc_l,yc_l] = c2u(xc_l_in,yc_l_in,xc_r_in,yc_r_in,obj.M_mpc);
           
           % Visibility Objective
           m_r = (yc_r - yr)/(xc_r - xr);
           m_l = (yc_l - yr)/(xc_l - xr);
           phi_r = atan(m_r);
           phi_l = atan(m_l);
           phi_r_y = phi_r/yr;
           phi_l_y = phi_l/yr;
           
           % Position Constraints
           if ~obj.last_sec
               m_r_inv = 1/m_r;
               m_l_inv = 1/m_l;
           else
               m_r_inv = 0;
               m_l_inv = 0;
           end
           
           if obj.lc_active
              perc_l_weight = 0.0005;
              xc_l = -50;
              yc_l = 0;
           else
              perc_l_weight = 0.0005;
              xc_l = -50;
              yc_l = 0;
           end
           upper_bound = 0;
           if obj.rc_active
               if obj.safe
                   perc_r_weight = 0.00005; %10000000;
                   x_weight = 10;
                   upper_bound = 100;
               else
                   perc_r_weight = 300;
                   x_weight = 10;
                   upper_bound = 0;
                   %%%%
%                    perc_r_weight = 0.00005; %10000000;
%                    x_weight = 10;
%                    upper_bound = 0;
               end
%               perc_r_weight = 300;
%               x_weight = 10;
           else
              perc_r_weight = 0.000005;
              x_weight = 50;
           end
           
           % If projected motion doesn't reach corner, don't set slope
           % constraint
           try
               x_last = obj.MPCinput.x(end,1); % Last (x,y) coordinate of projected motion
               y_last = obj.MPCinput.x(end,2);
               [~,y_last] = c2u(x_last,y_last,xc_r_in,yc_r_in,obj.M_mpc);
               if y_last < 0
                   m_inv = 0;
               end
           catch
           end
           
           var_r = phi_r_y;
           var_l = phi_l_y;
           var_des = 0;
           
           obj.MPCinput.x0 = [xr,yr,vx,vy,ax,ay,var_r,var_l,xc_l,yc_l,left_bound,right_bound,upper_bound,m_l_inv,m_r_inv,0];
           
           obj.MPCinput.x = [xr*ones((obj.N+1),1) ...
               yr*ones((obj.N+1),1) ...
               vx*ones(obj.N+1,1) ...
               vy*ones(obj.N+1,1) ...
               ax*ones(obj.N+1,1) ...
               ay*ones(obj.N+1,1) ...
               var_r*ones((obj.N+1),1) ...
               var_l*ones((obj.N+1),1) ...
               xc_l*ones((obj.N+1),1) ...
               yc_l*ones((obj.N+1),1) ...
               left_bound*ones(obj.N+1,1) ...
               right_bound*ones(obj.N+1,1) ...
               upper_bound*ones(obj.N+1,1) ...
               m_l_inv*ones(obj.N+1,1) ...
               m_r_inv*ones(obj.N+1,1) ...
               0*ones(obj.N+1,1)];
           
           obj.MPCinput.y = [xg*ones(obj.N,1), yg*ones(obj.N,1), var_r*ones(obj.N,1), var_l*ones(obj.N,1), 0*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1)];
           obj.MPCinput.yN = [xg yg var_des var_des];
           
           % x, y, perception right, perception left, ax_dot, ay_dot, epsilon
           A = diag([x_weight 50 perc_r_weight perc_l_weight 10 10 50000000]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
%            A_total = [];
%            for i= 1:obj.N
%               A_total = [A_total;diag([A(1,1),A(2,2)^(1+i*0.01),A(3,3),A(4,4),A(5,5),A(6,6),A(7,7)])]; 
%            end
%            obj.MPCinput.W = A_total;
           obj.MPCinput.WN = diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver7( obj.MPCinput );
           as = obj.MPCoutput.u(1,:);
           ax_dot = as(1);
           ay_dot = as(2);
           obj.ax = ax + ax_dot*dt;
           obj.ay = ay + ay_dot*dt;
           [ax,ay] = c2u(obj.ax,obj.ay,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = ax;
           obj.cmd_input.y = ay;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
           
       end
       function motion_step(obj)
           obj.vx = max(min(obj.vx,2),-2);
           obj.vy = max(min(obj.vy,2),-2);
           instate = [obj.x obj.y obj.vx obj.vy obj.cmd_input.x obj.cmd_input.y]';
           [t,outstate] = ode45(@(t,x) obj.EOM(t,x), [obj.t,obj.t+obj.dt], instate);
           obj.x = outstate(end,1);
           obj.y = outstate(end,2);
           obj.vx = outstate(end,3);
           obj.vy = outstate(end,4);
           obj.ax = obj.cmd_input.x;
           obj.ay = obj.cmd_input.y;
           obj.t = t(end);
           obj.rec_data();
       end
       function rec_data(obj)
           obj.trail.x = [obj.trail.x obj.x];
           obj.trail.y = [obj.trail.y obj.y];
           obj.vs.vxs = [obj.vs.vxs obj.vx];
           obj.vs.vys = [obj.vs.vys obj.vy];
           obj.cmd_inputs.x = [obj.cmd_inputs.x obj.cmd_input.x];
           obj.cmd_inputs.y = [obj.cmd_inputs.y obj.cmd_input.y];
           obj.rs = [obj.rs obj.r];
           obj.ts = [obj.ts obj.t];
%            obj.LOSs = [obj.LOSs obj.r];
       end
       function x_dot = EOM(obj,~,x)
           x_dot = zeros(6,1);
           x_dot(1) = x(3);
           x_dot(2) = x(4);
           x_dot(3) = x(5);
           x_dot(4) = x(6);
       end
   end
end