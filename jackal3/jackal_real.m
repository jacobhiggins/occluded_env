classdef jackal_real < handle
   properties
       R % wheel radius
      L % body length
      max_w % maximum wheel turn speed
      dd % differential drive object
      position = struct("x",0.0,"y",0.0);
      theta
      v % body frame linear vel
      w % body frame angular vel
      corner = struct("wypt",struct("x",0.0,"y",0.0),"mpc",struct("x",0.0,"y",0.0));
      xc_mpc_r
      yc_mpc_r
      xc_mpc_l
      yc_mpc_l
      lc_active = false; % False when left corner is not visible
      rc_active = false; % False when right corner is not visible
      xc_wp
      yc_wp
      wypt = struc('x',[],'y',[]);
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
      ku = struct("areas",[0.0]);
      sub;
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
          "is_true",false);
   end
   methods
       function init_params(obj)
           obj.sub = rossubscriber("/vicon/jackal3/jackal3");
           obj.pub = rospublisher("/jackal_velocity_controller/cmd_vel","geometry_msgs/Twist");
           obj.R = 0.075;
           obj.L = 0.42;
           obj.max_w = 26.667; % Max rotation for single wheel
           obj.max_omega = 1.5; % Max angular rotation for heading
           obj.max_v = 0.3;
           obj.size = 0.25; % m, max radius
           obj.dd = DifferentialDrive(obj.R,obj.L);
           obj.dt = 0.1;
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
       end
       function get_pose(obj)
           pos_msg = receive(obj.sub,1);
           obj.position.x = pos_msg.Transform.Translation.X;
           obj.position.y = pos_msg.Transform.Translation.Y;
       end
       function get_wypt(obj,map)
           obj.wypt.y = map.corner.y;
       end
       function get_corner(obj,map)
           obj.corner.mpc.x = map.corner.x;
           obj.corner.mpc.y = map.corner.y;
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
           vx_in = obj.cmd_input.x;
           vy_in = obj.cmd_input.y;
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
           A = diag([10 50 perc_r_weight 10 1 1000000]);
           
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
       function cmd_step(obj)
          persistent del_theta_int;
          if isempty(del_theta_int) || obj.t < 0.001
             del_theta_int = 0;
          end
          Kv = 1;
          Kw = 1;
          Kw_i = 0.001;
          vec = [obj.cmd_input.x;obj.cmd_input.y;0];
          heading = [cos(obj.theta);sin(obj.theta);0];
          v_cmd = Kv*max(min(norm(vec,2),obj.max_v),-obj.max_v);
          theta1 = obj.theta;
          theta2 = atan2(obj.cmd_input.y,obj.cmd_input.x);
          e_theta = theta2 - theta1;
          e_theta = atan2(sin(e_theta),cos(e_theta));
%           del_theta = sign(cross(vec,heading))*acos(dot(vec,heading)/norm(vec,2));
          del_theta_int = del_theta_int + e_theta;
          w_cmd = Kw*e_theta + Kw_i*del_theta_int;
          w_cmd = min(max(w_cmd,-obj.max_omega),obj.max_omega);
          if obj.debug.is_true
             obj.debug.v_cmds = [obj.debug.v_cmds;v_cmd];
             obj.debug.e_thetas = [obj.debug.e_thetas;e_theta];
             obj.debug.w_cmds = [obj.debug.w_cmds;w_cmd];
          end
          [wL,wR] = inverseKinematics(obj.dd,v_cmd,w_cmd);
          delw = max([wL,wR]-obj.max_w);
          wL = wL - delw;
          wR = wR - delw;
          tmp = max(min([wL,wR],obj.max_w),-obj.max_w);
          wL = tmp(1);
          wR = tmp(2);
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