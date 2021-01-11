classdef UGV < handle
   properties
      R % wheel radius
      L % body length
      max_w % maximum wheel turn speed
      dd % differential drive object
      x
      y
      theta
      v % body frame linear vel
      w % body frame angular vel
      xc_mpc_r
      yc_mpc_r
      xc_mpc_l
      yc_mpc_l
      lc_active = false; % False when left corner is not visible
      rc_active = false; % False when right corner is not visible
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
      max_v
      max_omega
      a_max
      size
      dt
      t
      r
      rs = [0.0];
      maxRad
      trail = struct('x',[],'y',[]); % Motion trail
      proj_mot = struct('x',[],'y',[]); % Projected motion
      cmd_input = struct('x',[0.0],'y',[0.0],'v',[0.0],'omega',[0.0],'name',"Velocities");
      cmd_inputs = struct('x',[0.0],'y',[0.0],'vs',[0.0],'omegas',[0.0]);
      vs = struct('vxs',[0.0],'vys',[0.0],'speeds',[0.0],'ws',[0.0]);
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
      ku = struct("areas",[0.0],"polys",[]);
      debug = struct("xrs",[],...
          "vxs",[],...
          "vys",[],...
          "x_accs",[],...
          "x_jerks",[],...
          "vx_cmds",[],...
          "vy_cmds",[],...
          "v_cmds",[],...
          "e_thetas",[],...
          "w_cmds",[],...
          "is_true",false);
      safe = false;
      stop_dist = 0.5;
   end
   methods
       function setParams(obj,map)
           clear obj.getcorner_MPC
           clear obj.getcorner_WP
           clear obj.get_wypt
           clear obj.motion_step
           obj.R = 0.038;
           obj.L = 0.354;
           obj.max_w = 17.11;
           obj.dd = DifferentialDrive(obj.R,obj.L);
           obj.dt = 0.01;
           obj.t = 0.0;
           obj.x = map.pose_start.x;
           obj.y = map.pose_start.y;
           obj.theta = pi/2;
           obj.v = 0;
           obj.w = 0;
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
           if isempty(curr_c) || obj.t < 0.001
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
           if isempty(dists) || obj.t < 0.001
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
           % NEW CODE, 09/02/2020
           %Moving waypoint to edge of FOV
%            del_vec = [waypoint.x - obj.x;waypoint.y - obj.y];
%            del_vec = obj.maxRad*del_vec/norm(del_vec,2);
%            obj.wypt.x = obj.x + del_vec(1);
%            obj.wypt.y = obj.y + del_vec(2);
           % NEW CODE, 09/02/2020
           obj.shift_wypt(map);
       end
       function get_wypt2(obj,map)
           % New method of setting placement of waypoint
           % Always at edge of FOV
           theta = atan2(obj.yc_wp - obj.y,obj.xc_wp - obj.x);
           waypoint.y = obj.x + obj.maxRad*cos(theta);
           waypoint.x = obj.y + obj.maxRad*sin(theta);
           obj.wypt = waypoint;
       end
       function scale_wypt(obj,map)
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
           dist = norm([x_avg;y_avg]-[obj.x;obj.y]);
           theta = atan2(obj.wypt.y - obj.y,obj.wypt.x - obj.x);
           obj.wypt.x = obj.x + dist*cos(theta);
           obj.wypt.y = obj.y + dist*sin(theta);
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
       
       function dist = expected_dist2coll(obj,map,test_point)
           num_points = 80;
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
%                        r = min(obj.r + obj.maxRad/50,r);
                        r = obj.maxRad;
                   else
%                        r = obj.r + obj.maxRad/50;
                       r = obj.maxRad;
                   end
               end
               r = min(dist_frac*obj.maxRad,r);
               if r > obj.maxRad
%                    r = min(obj.r + obj.maxRad/50,obj.maxRad);
                    r = obj.maxRad;
               end
           end
           obj.r = r;
       end
       function mpc_step2(obj)
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           upper_bound = 10;
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
           m_r_inv = 1/m_r;
           m_l_inv = 1/m_l;
           
           if obj.lc_active
              perc_l_weight = 5; 
           else
              perc_l_weight = 5;
              xc_l = -50;
              yc_l = 0;
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
           A = diag([50 500 5 perc_l_weight 250 250 50000000]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver7( obj.MPCinput );
           as = obj.MPCoutput.u(1,:);
           ax_dot = as(1);
           ay_dot = as(2);
           vx_cmd = vx + ax*dt + 0.5*ax_dot*dt^2;
           vy_cmd = vy + ay*dt + 0.5*ay_dot*dt^2;
           if(abs(vx_cmd)>0.001)
              obj.debug.is_true = true;
              obj.debug.xrs = [obj.debug.xrs;xr];
              obj.debug.vxs = [obj.debug.vxs;vx];
              obj.debug.vys = [obj.debug.vys;vy];
              obj.debug.x_accs = [obj.debug.x_accs;ax];
              obj.debug.x_jerks = [obj.debug.x_jerks;ax_dot];
              obj.debug.vx_cmds = [obj.debug.vx_cmds;vx_cmd];
              obj.debug.vy_cmds = [obj.debug.vy_cmds;vy_cmd];
           end
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = vx_cmd;
           obj.cmd_input.y = vy_cmd;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
           
       end
       function mpc_acc2vel(obj)
           if mod(obj.t,obj.dt) > 0.1001
               return;
           end
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           upper_bound = 10;
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
           dt = obj.dt;
           
           [xr,yr] = c2u(xr_in,yr_in,xc_r_in,yc_r_in,obj.M_mpc);
           [xg,yg] = c2u(xg_in,yg_in,xc_r_in,yc_r_in,obj.M_mpc);
           [vx,vy] = c2u(vx_in,vy_in,0,0,obj.M_mpc);
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
           m_r_inv = 1/m_r;
           m_l_inv = 1/m_l;
           
           if obj.current_sec==7
              m_r_inv = 0; 
           end
           
           if obj.lc_active
              perc_l_weight = 5; 
           else
              perc_l_weight = 5;
              xc_l = -50;
              yc_l = 0;
           end
           
           upper_bound = 0;
           if obj.rc_active
               if obj.safe
                   perc_r_weight = 0.00000005; %10000000;
                   x_weight = 10;
                   upper_bound = 100;
               else
                   perc_r_weight = 500;
                   x_weight = 10;
                   upper_bound = 0;
                   %%%%
%                    perc_r_weight = 0.00005; %10000000;
%                    x_weight = 10;
%                    upper_bound = 0;
               end
              perc_r_weight = 10000;
%               perc_r_weight = 0.00005;
%               x_weight = 5;
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
           
           obj.MPCinput.x0 = [xr,yr,vx,vy,var_r,var_l,xc_l,yc_l,left_bound,right_bound,upper_bound,m_l_inv,m_r_inv,0];
           
           obj.MPCinput.x = [xr*ones((obj.N+1),1) ...
               yr*ones((obj.N+1),1) ...
               vx*ones(obj.N+1,1) ...
               vy*ones(obj.N+1,1) ...
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
           
           % x, y, perception right, perception left, ax, ay, epsilon
           A = diag([x_weight 50 perc_r_weight perc_l_weight 25 25 500]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver_acc_cmd( obj.MPCinput );
           as = obj.MPCoutput.u(1,:);
           ax = as(1);
           ay = as(2);
           vx_cmd = vx + ax*dt;
           vy_cmd = vy + ay*dt;
           if(abs(vx_cmd)>0.001)
              obj.debug.is_true = true;
              obj.debug.xrs = [obj.debug.xrs;xr];
              obj.debug.vxs = [obj.debug.vxs;vx];
              obj.debug.vys = [obj.debug.vys;vy];
              obj.debug.x_accs = [obj.debug.x_accs;ax];
%               obj.debug.x_jerks = [obj.debug.x_jerks;ax_dot];
              obj.debug.vx_cmds = [obj.debug.vx_cmds;vx_cmd];
              obj.debug.vy_cmds = [obj.debug.vy_cmds;vy_cmd];
           end
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = vx_cmd;
           obj.cmd_input.y = vy_cmd;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
       end
       function mpc_stepvel(obj)
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
           dt = obj.dt;
           
           [xr,yr] = c2u(xr_in,yr_in,xc_r_in,yc_r_in,obj.M_mpc);
           [xg,yg] = c2u(xg_in,yg_in,xc_r_in,yc_r_in,obj.M_mpc);
           [vx,vy] = c2u(vx_in,vy_in,0,0,obj.M_mpc);
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
              perc_l_weight = 5; 
           else
              perc_l_weight = 5;
              xc_l = -50;
              yc_l = 0;
           end
           if obj.rc_active
              perc_r_weight = 5; 
           else
              perc_r_weight = 5;
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
           
           obj.MPCinput.x0 = [xr,yr,var_r,xc_l,yc_l,left_bound,right_bound,m_l_inv,m_r_inv,0];
           
           obj.MPCinput.x = [xr*ones((obj.N+1),1), ...
               yr*ones((obj.N+1),1), ...
               var_r*ones(obj.N+1,1), ...
               xc_l*ones((obj.N+1),1), ...
               yc_l*ones((obj.N+1),1), ...
               left_bound*ones(obj.N+1,1), ...
               right_bound*ones(obj.N+1,1), ...
               m_l_inv*ones(obj.N+1,1), ...
               m_r_inv*ones(obj.N+1,1), ...
               0*ones(obj.N+1,1)];
           
           %                var_r*ones((obj.N+1),1) ...
%                var_l*ones((obj.N+1),1) ...
           
           
           obj.MPCinput.y = [xg*ones(obj.N,1), ...
               yg*ones(obj.N,1), ...
               0*ones(obj.N,1), ...
               0*ones(obj.N,1), ...
               0*ones(obj.N,1), ...
               0*ones(obj.N,1)];
           
           %                var_des*ones(obj.N,1), 
%                var_des*ones(obj.N,1),

           obj.MPCinput.yN = [xg yg]; 
%                var_des var_des
%                ];
           
           % x, y, phi_right, vx, vy, epsilon
           A = diag([10 50 perc_r_weight 10 1 1]);
%            A = diag([500 50 perc_r_weight 10 1 1000000]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = diag([A(1,1) A(2,2)]);% A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver_vel_cmd( obj.MPCinput );
           us = obj.MPCoutput.u(1,:);
           vx_cmd = us(1);
           vy_cmd = us(2);
           
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = vx_cmd;
           obj.cmd_input.y = vy_cmd;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
       end
       function test_step(obj)
           vec = [obj.wypt.x-obj.x;obj.wypt.y-obj.y];
           vec = 10*vec/norm(vec,2);
           obj.cmd_input.x = vec(1);
           obj.cmd_input.y = vec(2);
       end
       function motion_step(obj,sim_dt)
          persistent del_theta_int;
          persistent e_theta_old;
          if isempty(del_theta_int) || obj.t < 0.001
             del_theta_int = 0;
             e_theta_old= 0;
          end
          Kv = 1;
%           Kw = 1; % Before 09/29/2020
%           Kw_i= 0.001;
          Kw = 10;
          Kw_i = 0.01;
          Kw_d = 1;
          vec = [obj.cmd_input.x;obj.cmd_input.y];
          v_cmd = Kv*max(min(norm(vec,2),obj.max_v),-obj.max_v);
          theta1 = obj.theta;
          theta2 = atan2(obj.cmd_input.y,obj.cmd_input.x);
          e_theta = theta2 - theta1;
          e_theta = atan2(sin(e_theta),cos(e_theta));
%           del_theta = sign(cross(vec,heading))*acos(dot(vec,heading)/norm(vec,2));
          del_theta_int = del_theta_int + e_theta;
          e_theta_dot = (e_theta - e_theta_old)/obj.dt;
          e_theta_old = e_theta;
          w_cmd = Kw*e_theta + Kw_i*del_theta_int + Kw_d*e_theta_dot;
          w_cmd = min(max(w_cmd,-obj.max_omega),obj.max_omega);
          if obj.debug.is_true
             obj.debug.v_cmds = [obj.debug.v_cmds;v_cmd];
             obj.debug.e_thetas = [obj.debug.e_thetas;e_theta];
             obj.debug.w_cmds = [obj.debug.w_cmds;w_cmd];
          end
          [wL,wR] = inverseKinematics(obj.dd,v_cmd,w_cmd);
          if (max(wL,wR) > obj.max_w)
              delw = max([wL,wR]-obj.max_w);
              wL = wL - delw;
              wR = wR - delw;
              tmp = max(min([wL,wR],obj.max_w),-obj.max_w);
              wL = tmp(1);
              wR = tmp(2);
          end
          [v,w] = forwardKinematics(obj.dd,wL,wR);
          velB = [v;0;w]; % Body velocities [vx;vy;w]
          vel = bodyToWorld(velB,[obj.x;obj.y;obj.theta]);  % Convert from body to world
          obj.ax = (vel(1) - obj.vx)/sim_dt;
          obj.ay = (vel(2) - obj.vy)/sim_dt;
          obj.vx = vel(1);
          obj.vy = vel(2);
          % Perform forward discrete integration step
          pose = [obj.x;obj.y;obj.theta] + vel*sim_dt;
          obj.x = pose(1);
          obj.y = pose(2);
          obj.theta = atan2(sin(pose(3)),cos(pose(3)));
          obj.v = v;
          obj.w = w;
          obj.cmd_input.v = v;
          obj.cmd_input.omega = w;
          obj.t = obj.t + sim_dt;
          obj.rec_data();
       end
       function motion_stepDD(obj)
          v_cmd = obj.cmd_inputs.v;
          w_cmd = obj.cmd_inputs.omega;
          [wL,wR] = inverseKinematics(obj.dd,v_cmd,w_cmd);
          [v,w] = forwardKinematics(obj.dd,wL,wR);
          velB = [v;0;w]; % Body velocities [vx;vy;w]
          vel = bodyToWorld(velB,[obj.x;obj.y;obj.theta]);  % Convert from body to world
          pose = [obj.x;obj.y;obj.theta] + vel*obj.dt;
          obj.x = pose(1);
          obj.y = pose(2);
          obj.theta = sign(pose(3))*mod(pose(3),2*pi);
          obj.v = v;
          obj.w = w;
          obj.vx = v*cos(obj.theta);
          obj.vy = v*sin(obj.theta);
          obj.t = obj.t + obj.dt;
          obj.rec_data();
       end
       function rec_data(obj)
           obj.trail.x = [obj.trail.x obj.x];
           obj.trail.y = [obj.trail.y obj.y];
           obj.vs.vxs = [obj.vs.vxs obj.vx];
           obj.vs.vys = [obj.vs.vys obj.vy];
           obj.vs.speeds = [obj.vs.speeds obj.v];
           obj.vs.ws = [obj.vs.ws obj.w];
           obj.cmd_inputs.x = [obj.cmd_inputs.x obj.cmd_input.x];
           obj.cmd_inputs.y = [obj.cmd_inputs.y obj.cmd_input.y];
           obj.cmd_inputs.vs = [obj.cmd_inputs.vs obj.cmd_input.v];
           obj.cmd_inputs.omegas = [obj.cmd_inputs.omegas obj.cmd_input.omega];
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