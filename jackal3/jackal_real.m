classdef jackal_real < handle
   properties
       R % wheel radius
       L % body length
       max_w % maximum wheel turn speed
       dd % differential drive object
       position = struct("x",0.0,"y",0.0);
       vels = struct("v",0.0,"x",0.0,"y",0.0);
       theta
       v % body frame linear vel
       w % body frame angular vel
       corner = struct("wypt",struct("x",0.0,"y",0.0),"mpc",struct("x",0.0,"y",0.0));
       corner_hist = {};
       xc_mpc_r
       yc_mpc_r
       xc_mpc_l
       yc_mpc_l
       lc_active = false; % False when left corner is not visible
       rc_active = false; % False when right corner is not visible
       xc_wp
       yc_wp
       wypt = struc('x',[],'y',[]);
       wypt_hist = {};
       M_mpc = eye(2);
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
       FOV = struct("radius",0.0,"max_radius",0.0);
       r
       rs = [0.0];
       maxRad
       trail = struct('x',[],'y',[],'theta',[]); % Motion trail
       proj_mot = struct('x',[],'y',[]); % Projected motion
       proj_mots = [];
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
       N = 40; % Control Horizon
       Nu = 3; % Decision variables
       MPCinput = struct('x0',[],'x',[],'y',[],'yN',[],'W',[],'WN',[]);
       MPCoutput = struc('x',[],'u',[]);
       large_num = 100000;
       ku = struct("areas",[0.0],"poly",struct("x",[-100 -100 -99 -99],"y",[-100 -99 -99 -100]));
       kus = [];
       sub_pos;
       sub_vel;
       sub_joy;
       pub;
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
           "mpc_delt",[],...
           "mpc_t",[],...
           "ll_t",[],...
           "is_true",false);
       width = 0.43;
       length = 0.508;
       joy_buttons = [];
       JOY_X = 1;
       JOY_O = 2;
       outline = struct("x",[],"y",[]);
       outlines = [];
       last_sec = false;
       MPC_Hz = 8; % OG: 10
   end
   methods
       function init_params(obj)
           obj.sub_pos = rossubscriber("/vicon/jackal3/jackal3");
           obj.sub_vel = rossubscriber("/odometry/filtered");
           obj.sub_joy = rossubscriber("/bluetooth_teleop/joy");
           obj.pub = rospublisher("/jackal_velocity_controller/cmd_vel","geometry_msgs/Twist");
           obj.get_pose();
           obj.R = 0.075;
           obj.L = 0.42;
           obj.max_w = 26.667; % Max rotation for single wheel
           obj.max_omega = 1.0; % Max angular rotation for heading
           obj.max_v = 1.5;
           obj.size = 0.25; % m, max radius
           obj.dd = DifferentialDrive(obj.R,obj.L);
           obj.dt = 1/obj.MPC_Hz;
           obj.t = 0.0;
           obj.get_pose();
           obj.theta = pi/2;
           obj.v = 0;
           obj.w = 0;
           obj.ax = 0;
           obj.ay = 0;
           obj.MPCinput.u = zeros(obj.N,obj.Nu);
           obj.trail.x = obj.position.x;
           obj.trail.y = obj.position.y;
           obj.proj_mot.x = obj.position.x*ones(1,obj.N);
           obj.proj_mot.y = obj.position.y*ones(1,obj.N);
           obj.wypt.x = obj.position.x;
           obj.wypt.y = obj.position.y;
           obj.r = 3;
           obj.maxRad = 3;
           obj.outline = obj.square_points();
       end
       function get_joy(obj)
           joy_msg = receive(obj.sub_joy,1);
           obj.joy_buttons = joy_msg.Buttons;
       end
       function get_pose(obj)
           persistent old_pos
           persistent last_sub_time;
           if isempty(old_pos)
              pos_msg = receive(obj.sub_pos,1);
              old_pos.x = pos_msg.Transform.Translation.X;
              old_pos.y = pos_msg.Transform.Translation.Y;
              old_pos.t = toc;
           end
           if isempty(last_sub_time)
              last_sub_time = toc; 
              pause(1/obj.MPC_Hz);
           end
           if (toc - last_sub_time) < 1/obj.MPC_Hz
              return;
           end
           pos_msg = receive(obj.sub_pos,1);
%            vel_msg = receive(obj.sub_vel,1);
           obj.position.x = pos_msg.Transform.Translation.X;
           obj.position.y = pos_msg.Transform.Translation.Y;
           obj.t = toc;
           obj.vels.v = norm([obj.position.x-old_pos.x;obj.position.y-old_pos.y])/( (obj.t-old_pos.t));
%            v = vel_msg.Twist.Twist.Linear.X;
           q = [pos_msg.Transform.Rotation.W,...
               pos_msg.Transform.Rotation.X,...
               pos_msg.Transform.Rotation.Y,...
               pos_msg.Transform.Rotation.Z];
           R = quat2rotm(q);
           if abs(obj.vels.y) < 0.1 && abs(obj.vels.x) < 0.1
               obj.theta = pi/2;
%                obj.theta = -atan2(R(1,2),R(1,1)) + pi/2; %% Get correct angle from rotations
               % I think orientation of jackal3 defined in vicon is a
               % little bit off, makes jackal not go straight...
           pause(1/obj.MPC_Hz); 
           else
               obj.theta = atan2(obj.vels.y,obj.vels.x);
           end
           obj.theta = atan2(sin(obj.theta),cos(obj.theta)); %% Fit heading within [-pi,pi]
           %            obj.vels.x = obj.vels.v*cos(obj.theta);
%            obj.vels.y = obj.vels.v*sin(obj.theta);
           obj.vels.x = (obj.position.x-old_pos.x)/(obj.t-old_pos.t);
           obj.vels.y = (obj.position.y-old_pos.y)/(obj.t-old_pos.t);
           old_pos.x = obj.position.x;
           old_pos.y = obj.position.y;
           old_pos.t = obj.t;
           last_sub_time = obj.t;
       end
       function get_wypt(obj,map)
%            obj.wypt.y = map.corner.y;
%             obj.wypt.x = obj.corner.mpc.x - 2.5/2;
%             obj.wypt.y = 0;
%             obj.wypt.x = 0.5;
%             obj.wypt.y = 1;
           persistent dists; % Distances between waypoints
           persistent vecs; % vectors between waypoints
           persistent c_base; % current waypoint base
           if isempty(dists) || obj.t < 0.001
               for i = 1:length(map.model_traj.wypt_bases.xs)-1
                   wypta = [map.model_traj.wypt_bases.xs(i),...
                       map.model_traj.wypt_bases.ys(i)];
                   wyptb = [map.model_traj.wypt_bases.xs(i+1),...
                       map.model_traj.wypt_bases.ys(i+1)];
                   vec = wyptb - wypta;
                   vec = vec/norm(vec,2);
                   dists = [dists;norm(wypta-wyptb,2)];
                   vecs = [vecs;vec];
                   c_base = 1;
               end
           end
           c_base = min(length(dists),c_base); % current waypoint index for circle
           wpb = [map.model_traj.wypt_bases.xs(c_base);...
               map.model_traj.wypt_bases.ys(c_base)]; % current waypoint index for radius
           next_wpb = [map.model_traj.wypt_bases.xs(min(c_base+1,length(map.model_traj.wypt_bases.xs)));...
               map.model_traj.wypt_bases.ys(min(c_base+1,length(map.model_traj.wypt_bases.xs)))]; % next waypoint for circle
%            next_next_wpb = [map.model_traj.wypt_bases.xs(min(c_base+2,size(map.wypt_bases,1)));...
%                map.model_traj.wypt_bases.ys(min(c_base+2,size(map.wypt_bases,1)))]; % next next waypoint for circle
           vec = vecs(c_base,:); % vector between current and next waypoint for circle
           m = vec(2)/vec(1); % slope of line bewteen current and next waypoint for circle
           
           % Determination of radius using appropriate waypoints
           n = length(map.model_traj.wypt_bases.xs);
           xm2 = map.model_traj.wypt_bases.xs(min(obj.current_sec+1,n)); % ALSO CHANGED
           ym2 = map.model_traj.wypt_bases.ys(min(obj.current_sec+1,n));
           xm3 = map.model_traj.wypt_bases.xs(min(obj.current_sec+2,n));
           ym3 = map.model_traj.wypt_bases.ys(min(obj.current_sec+2,n));
           theta2 = atan2(ym3-ym2,xm3-xm2);
           
           % Set radius using appropriate waypoints
           obj.set_radius(xm2,ym2,theta2,1);
           
           [xis,yis] = linecirc(m,0,obj.position.x-wpb(1),obj.position.y-wpb(2),obj.r); % x's and y's where circle intersects line
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
           if norm([obj.position.x-next_wpb(1),obj.position.y-next_wpb(2)],2) < obj.r
               c_base = c_base + 1;
           end
       end
       function get_wypt_simple(obj,map)
           persistent check;
           if isempty(check)
               obj.wypt.x = obj.position.x;
               obj.wypt.y = obj.position.y + 5;
               check = 1;
           end
       end
       function set_radius(obj,xm2,ym2,theta2,dist_frac)
           r = obj.maxRad;
           if(obj.position.y > 0) % TODO: fix bug in radius of 2nd hallway
               obj.r = r;
               return
           end
           theta = atan2(obj.corner.mpc.y-obj.position.y,obj.corner.mpc.x-obj.position.x); % TODO: change mpc corner to wypt corner
           if theta ~= theta2
               A = [cos(theta),-cos(theta2);...
                   sin(theta),-sin(theta2)];
               b = [xm2-obj.position.x;ym2-obj.position.y];
               vec = A\b;
               r = vec(1);
               if (r > obj.r && r < obj.maxRad) || r < 0
                   if r > 0
%                        r = min(obj.r + obj.maxRad/50,r);
%                         r = obj.maxRad;
                   else
%                        r = obj.r + obj.maxRad/50;
                       r = obj.maxRad;
                   end
               end
               r = min(obj.maxRad,r);
               if r > obj.maxRad
%                    r = min(obj.r + obj.maxRad/50,obj.maxRad);
                    r = obj.maxRad;
               end
           end
           obj.r = r;
       end
       function get_corner(obj,map)
           persistent curr_rc;
           if isempty(curr_rc)
               curr_rc = 1;
               obj.corner.mpc.x = map.geoms{curr_rc}.corner.x- max(obj.width,obj.length)/2;
               obj.corner.mpc.y = map.geoms{curr_rc}.corner.y;
               obj.M_mpc = map.geoms{curr_rc}.M_mpc;
               obj.corner_hist{1} = obj.corner;
           end
           
           obj.corner.mpc.x = map.geoms{curr_rc}.corner.x - max(obj.width,obj.length)/2;
           obj.corner.mpc.y = map.geoms{curr_rc}.corner.y;
           obj.M_mpc = map.geoms{curr_rc}.M_mpc;
           
           [~,dely_rc] = c2u(obj.position.x,obj.position.y,obj.corner.mpc.x,obj.corner.mpc.y,obj.M_mpc);
%            [~,dely_lc] = c2u(obj.x,obj.y,obj.xc_mpc_l,obj.yc_mpc_l,M);
           
           if dely_rc > 0
               curr_rc = min(curr_rc+1,length(map.geoms));
               obj.corner.mpc.x = map.geoms{curr_rc}.corner.x - max(obj.width,obj.length)/2;
               obj.corner.mpc.y = map.geoms{curr_rc}.corner.y;
               obj.M_mpc = map.geoms{curr_rc}.M_mpc;
               obj.corner_hist{curr_rc} = obj.corner
           end
%            
%            obj.corner.mpc.x = obj.corner.mpc.x; % Prevent jackal from hitting wall
%            obj.corner.mpc.y = obj.corner.mpc.y;
       end
       function pub_cmd(obj)
          persistent pub_msg;
          if isempty(pub_msg)
              pub_msg = rosmessage(obj.pub);
          end
          pub_msg.Linear.X = obj.cmd_input.v;
          pub_msg.Angular.Z = obj.cmd_input.omega;
          send(obj.pub,pub_msg);
       end
       function mpc_stepvel(obj)
           persistent last_sample_time;
           if isempty(last_sample_time)
              last_sample_time = toc; 
           end
           if obj.t - last_sample_time < 0.95/obj.MPC_Hz
               return;
           end
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           if obj.current_sec==3
               right_bound = 15;
           end
           xr_in = obj.position.x;
           yr_in = obj.position.y;
           xc_r_in = obj.corner.mpc.x;
           yc_r_in = obj.corner.mpc.y;
           xc_l_in = -10;
           yc_l_in = 0;
           xg_in = obj.wypt.x;
           yg_in = obj.wypt.y;
           vx_in = obj.vels.x;
           vy_in = obj.vels.y;
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
           
           if obj.lc_active
              perc_l_weight = 0.0005; 
           else
              perc_l_weight = 5;
              xc_l = -50;
              yc_l = 0;
           end
%            if obj.rc_active
              perc_r_weight = 500; 
%            else
%               perc_r_weight = 0.0005;
%            end
%            perc_r_weight = 0.00005;
%            perc_r_weight = 50;
           
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
           A = diag([50 50 perc_r_weight 10 1 1000000]); % 1000000
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = diag([A(1,1) A(2,2)]);% A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver_vel_cmd_jackal3( obj.MPCinput );
           us = obj.MPCoutput.u(1,:);
           vx_cmd = us(1);
           vy_cmd = us(2);
           
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = vx_cmd;
           obj.cmd_input.y = vy_cmd;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
           
           last_sample_time = toc;
       end
       
       function mpc_acc2vel(obj)
           persistent last_sample_time;
           if isempty(last_sample_time)
              last_sample_time = toc-1.1/obj.MPC_Hz; 
           end
           if toc - last_sample_time < 1/obj.MPC_Hz
               return;
           end
           start_mpc_time = toc;
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           upper_bound = obj.wypt.y - obj.corner.mpc.y;
%            upper_bound = 10;
           if obj.current_sec==3
               right_bound = 15;
           end
           xr_in = obj.position.x;
           yr_in = obj.position.y;
           xc_r_in = obj.corner.mpc.x;
           yc_r_in = obj.corner.mpc.y;
           xc_l_in = -10;
           yc_l_in = 0;
           xg_in = obj.wypt.x;
           yg_in = obj.wypt.y;
           vx_in = obj.vels.x;
           vy_in = obj.vels.y;
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
           
           if obj.lc_active
              perc_l_weight = 0.00005; 
           else
              perc_l_weight = 0.00005;
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
           A = diag([50 50 0.000005 perc_l_weight 100 100 50000000]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
           obj.MPCinput.WN = diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
           
           obj.MPCoutput = acado_solver_acc_cmd_jackal3( obj.MPCinput );
           as = obj.MPCoutput.u(1,:);
           ax = as(1);
           ay = as(2);
%            vx_cmd = vx + ax*dt;
%            vy_cmd = vy + ay*dt;
           vx_cmd = obj.MPCoutput.x(2,3);
           vy_cmd = obj.MPCoutput.x(2,4);
           if true
              obj.debug.is_true = true;
              obj.debug.xrs = [obj.debug.xrs;xr];
              obj.debug.vxs = [obj.debug.vxs;vx];
              obj.debug.vys = [obj.debug.vys;vy];
              obj.debug.x_accs = [obj.debug.x_accs;ax];
%               obj.debug.x_jerks = [obj.debug.x_jerks;ax_dot];
              obj.debug.vx_cmds = [obj.debug.vx_cmds;vx_cmd];
              obj.debug.vy_cmds = [obj.debug.vy_cmds;vy_cmd];
              obj.debug.mpc_t = [obj.debug.mpc_t;toc];
           end
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(obj.M_mpc));
           obj.cmd_input.x = vx_cmd;
           obj.cmd_input.y = vy_cmd;
           
           proj_mot = (inv(obj.M_mpc)*obj.MPCoutput.x(:,1:2)' + [xc_r_in*ones(obj.N+1,1) yc_r_in*ones(obj.N+1,1)]')';
           
           obj.proj_mot.x = proj_mot(:,1);
           obj.proj_mot.y = proj_mot(:,2);
           last_sample_time = toc;
           if obj.debug.is_true
              obj.debug.mpc_delt = [obj.debug.mpc_delt; last_sample_time-start_mpc_time]; 
           end
       end
       
       function cmd_step(obj)
          persistent del_theta_int;
          persistent e_theta_old;
          persistent e_theta_old_t;
          persistent v_ref_weighted;
          persistent alpha;
          if isempty(del_theta_int) || obj.t < 0.001
             del_theta_int = 0;
             e_theta_old = 0;
             e_theta_old_t = toc;
             v_ref_weighted = 0;
             alpha = 0.5;
          end
          if norm([obj.cmd_input.x,obj.cmd_input.y]) < 0.00000001
              obj.cmd_input.v = 0;
              obj.cmd_input.omega = 0;
             return; 
          end
          Kw = 2;
          Kw_i = 1;
          Kw_d = 0.1;
          vec = [obj.cmd_input.x;obj.cmd_input.y;0];
%           vec = 1; % CHANGE BACK !!!!!!!
          heading = [cos(obj.theta);sin(obj.theta);0];
          
          v_ref = max(min(norm(vec,2),obj.max_v),-obj.max_v);
          v_ref_weighted = alpha*v_ref + (1-alpha)*v_ref_weighted;
          
          v_cmd = v_ref_weighted;
          theta1 = obj.theta;
          theta2 = atan2(obj.cmd_input.y,obj.cmd_input.x);
          e_theta = theta2 - theta1;
          e_theta = atan2(sin(e_theta),cos(e_theta));
%           del_theta = sign(cross(vec,heading))*acos(dot(vec,heading)/norm(vec,2));
          del_theta_int = del_theta_int + e_theta;
          e_theta_dot = (e_theta - e_theta_old)/(toc - e_theta_old_t);
          e_theta_old = e_theta;
          e_theta_old_t = toc;
          w_cmd = Kw*e_theta + Kw_i*del_theta_int * Kw_d*e_theta_dot;
          w_cmd = min(max(w_cmd,-obj.max_omega),obj.max_omega);
          if obj.debug.is_true
             obj.debug.v_cmds = [obj.debug.v_cmds; v_cmd];
             obj.debug.e_thetas = [obj.debug.e_thetas;e_theta];
             obj.debug.w_cmds = [obj.debug.w_cmds;w_cmd];
             obj.debug.ll_t = [obj.debug.ll_t;toc];
          end
          [wL,wR] = inverseKinematics(obj.dd,v_cmd,w_cmd);
%           delw = max([wL,wR]-obj.max_w);
%           wL = wL - delw;
%           wR = wR - delw;
%           tmp = max(min([wL,wR],obj.max_w),-obj.max_w);
%           wL = tmp(1);
%           wR = tmp(2);
          [v,w] = forwardKinematics(obj.dd,wL,wR);
          velB = [v;0;w]; % Body velocities [vx;vy;w]
          vel = bodyToWorld(velB,[obj.position.x;obj.position.y;obj.theta]);  % Convert from body to world
          obj.cmd_input.v = max(min(v,obj.max_v),-obj.max_v);
          obj.cmd_input.omega = max(min(w,obj.max_w),-obj.max_w);
          obj.rec_data();
       end
       function rec_data(obj)
           
           obj.trail.x = [obj.trail.x obj.position.x];
           obj.trail.y = [obj.trail.y obj.position.y];
           obj.trail.theta = [obj.trail.theta obj.theta];
           obj.proj_mots = [obj.proj_mots obj.proj_mot];
           obj.vs.vxs = [obj.vs.vxs obj.vels.x];
           obj.vs.vys = [obj.vs.vys obj.vels.y];
           obj.vs.speeds = [obj.vs.speeds obj.v];
           obj.vs.ws = [obj.vs.ws obj.w];
           obj.cmd_inputs.x = [obj.cmd_inputs.x obj.cmd_input.x];
           obj.cmd_inputs.y = [obj.cmd_inputs.y obj.cmd_input.y];
           obj.cmd_inputs.vs = [obj.cmd_inputs.vs obj.cmd_input.v];
           obj.cmd_inputs.omegas = [obj.cmd_inputs.omegas obj.cmd_input.omega];
           obj.rs = [obj.rs obj.r];
           obj.ts = [obj.ts obj.t];
           obj.outline = obj.square_points();
           obj.outlines{end+1} = obj.square_points();
           obj.wypt_hist{end+1} = obj.wypt;
%            obj.LOSs = [obj.LOSs obj.r];
       end
       function x_dot = EOM(obj,~,x)
           x_dot = zeros(6,1);
           x_dot(1) = x(3);
           x_dot(2) = x(4);
           x_dot(3) = x(5);
           x_dot(4) = x(6);
       end
       function points = square_points(obj)
           persistent local_points;
           if isempty(local_points)
               local_points = struct("x",[],"y",[]);
               % Bottom edge
               new_points_x = (-obj.width/2:0.01:obj.width/2);
               new_points_y = -obj.length/2*ones(size(new_points_x));
               local_points.x = [local_points.x new_points_x];
               local_points.y = [local_points.y new_points_y];
               % Right edge
               new_points_y = -obj.length/2:0.01:obj.length/2;
               new_points_x = obj.width/2*ones(size(new_points_y));
               local_points.x = [local_points.x new_points_x];
               local_points.y = [local_points.y new_points_y];
               % Top edge
               new_points_x = obj.width/2:-0.01:-obj.width/2;
               new_points_y = obj.length/2*ones(size(new_points_x));
               local_points.x = [local_points.x new_points_x];
               local_points.y = [local_points.y new_points_y];
               % Left edge
               new_points_y = obj.length/2:-0.01:-obj.length/2;
               new_points_x = -obj.width/2*ones(size(new_points_y));
               local_points.x = [local_points.x new_points_x];
               local_points.y = [local_points.y new_points_y];
           end
           
           rot_mat = [cos(obj.theta-pi/2) -sin(obj.theta-pi/2);...
               sin(obj.theta-pi/2) cos(obj.theta-pi/2)];
           
           translation = [obj.position.x;obj.position.y];
           
           global_points = rot_mat*[local_points.x;local_points.y] + translation;
           
           points.x = global_points(1,:);
           points.y = global_points(2,:);
%            points.rot_mat = rot_mat;
%            points.translation = translation;
%            points.local_points = local_points;
           
       end
       function stop(obj)
           obj.cmd_input.v = 0;
           obj.cmd_input.omega = 0;
           obj.pub_cmd();
       end
   end
end