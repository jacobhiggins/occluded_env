classdef point < handle
   properties
      x
      y
      xc_mpc
      yc_mpc
      xc_wp
      yc_wp
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
      % MPC vars
      N = 50; % Control Horizon
      Nu = 4; % Decision variables
      MPCinput = struct('x0',[],'x',[],'y',[],'yN',[],'W',[],'WN',[]);
      MPCoutput = struc('x',[],'u',[]);
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
           obj.xc_mpc = map.corners(1,1);
           obj.yc_mpc = map.corners(1,2);
           obj.xc_wp = map.corners(1,1);
           obj.yc_wp = map.corners(1,2);
           obj.wypt.x = obj.x;
           obj.wypt.y = obj.y + obj.maxRad;
           obj.MPCinput.u = zeros(obj.N,obj.Nu);
           obj.trail.x = obj.x;
           obj.trail.y = obj.y;
           obj.proj_mot.x = obj.x*ones(1,obj.N);
           obj.proj_mot.y = obj.y*ones(1,obj.N);
           obj.maxRad = map.maxRad_suggest;
           obj.r = obj.maxRad;
       end
       function getcorner_MPC(obj,map)
          persistent curr_cs;
            if isempty(curr_cs)
                curr_cs = 1;
            end
            M = map.Ms_mpc{curr_cs};
            [~,dely] = c2u(obj.x,obj.y,obj.xc_mpc,obj.yc_mpc,M);
            if dely > 0
                curr_cs = min(curr_cs + 1,size(map.corners,1));
            end
            obj.xc_mpc = map.corners(curr_cs,1);
            obj.yc_mpc = map.corners(curr_cs,2);
            obj.M_mpc = map.Ms_mpc{curr_cs};
       end
       function getcorner_WP(obj,map)
           persistent curr_c; % current corner index
           persistent next_c;
           if isempty(curr_c)
               curr_c = 1;
               next_c = 2;
           end
           curr_c = min(curr_c,size(map.corners,1));
           next_c = min(next_c,size(map.corners,1));
           xc1 = map.corners(curr_c,1);
           yc1 = map.corners(curr_c,2);
           xc2 = map.corners(next_c,1);
           yc2 = map.corners(next_c,2);
           avoid = map.corners(curr_c,3);
           phi1 = atan2(yc1-obj.y,xc1-obj.x);
           phi2 = atan2(yc2-obj.y,xc2-obj.x);
           if sign(phi2 - phi1) == avoid
               curr_c = curr_c + 1;
               next_c = next_c + 1;
           end
           obj.xc_wp = map.corners(curr_c,1);
           obj.yc_wp = map.corners(curr_c,2);
%            M = Ms{curr_c};
%            flip = corners(curr_c,3);
           obj.current_sec = curr_c;
           obj.current_owall = map.walls_mpc(curr_c);
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
       end
       function set_radius(obj,xm2,ym2,theta2,dist_frac)
           r = obj.maxRad;
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
           xc_in = obj.xc_mpc;
           yc_in = obj.yc_mpc;
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
               [~,y_last] = c2u(x_last,y_last,xc_in,yc_in,M);
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
       function motion_step(obj)
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