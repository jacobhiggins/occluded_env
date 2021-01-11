classdef UGV < AMR
   properties
      physical = struct(...
          "L",0.0,...
          "W",0.0,...
          "wheel_rad",0.0,...
          "v_max",0.0,...
          "omega_max",0.0...
          );
      theta = pi/2;
      dd;
   end
   methods
       function obj = UGV(map)
          obj = obj@AMR(map);
          obj.outer_cmd.names = ["vx","vy"];
          obj.inner_cmd.names = ["v","omega"];
       end
       
       function set_MPC_weights(obj)
           
       end
       function mpc_a2v(obj)
           pos_in = [obj.position.x obj.position.y];
           vel_in = [obj.vel.x obj.vel.y];
           acc_in = [obj.acc.x obj.acc.y];
           wypt_in = [obj.MPC_vals.wypt.x obj.MPC_vals.wypt.y];
           right_corner = [obj.MPC_vals.right_corner.x,obj.MPC_vals.right_corner.y];
           M = obj.MPC_vals.M;
           % Transform
           [x,y] = c2u(pos_in(1),pos_in(2),right_corner(1),right_corner(2),M);
           [vx,vy] = c2u(vel_in(1),vel_in(2),0.0,0.0,M);
           [xg,yg] = c2u(wypt_in(1),wypt_in(2),right_corner(1),right_corner(2),M);
           % Slopes
           m_r = y/x;
           if obj.MPC_vals.right_corner.active
              m_r_inv = x/y;
           else
              m_r_inv = 0; 
           end
           phi_r = atan(m_r);
           phi_r_y = phi_r/y;
           phi_r_y_des = 0.0;
           % Setup MPC values
           left_bound = -5;
           right_bound = 0.0;
           upper_bound = obj.MPC_vals.lims.upper;
           obj.acado_vals.MPC_input.u = zeros(obj.acado_vals.CH,obj.acado_vals.input_num);
           obj.acado_vals.MPC_input.x0 = [x,y,vx,vy,phi_r_y,left_bound,right_bound,upper_bound,m_r_inv,10];
           obj.acado_vals.MPC_input.x = repmat(obj.acado_vals.MPC_input.x0,obj.acado_vals.CH+1,1);
           obj.acado_vals.MPC_input.y = repmat([xg, yg, phi_r_y_des,0,0,0],obj.acado_vals.CH,1);
           obj.acado_vals.MPC_input.yN = [xg yg phi_r_y_des];
           % Set MPC weights
           % x, y, perception right, perception left, ax, ay, epsilon
           x_weight = 10;
           perc_r_weight = 0.01;
           epsilon_weight = 1000000000000;
           A = diag([x_weight 50 perc_r_weight 25 25 epsilon_weight]);
           obj.acado_vals.MPC_input.W = repmat(A,obj.acado_vals.CH,1);
           obj.acado_vals.MPC_input.WN = diag([A(1,1) A(2,2) A(3,3) ]);
           % Solve
           obj.acado_vals.MPC_output = acado_solver_acc_cmd( obj.acado_vals.MPC_input );
           as = obj.acado_vals.MPC_output.u(1,:);
           ax = as(1);
           ay = as(2);
           vx_cmd = vx + ax/obj.control_Hz.outer;
           vy_cmd = vy + ay/obj.control_Hz.outer;
           [vx_cmd,vy_cmd] = c2u(vx_cmd,vy_cmd,0,0,inv(M));
           obj.outer_cmd.vals = [vx_cmd,vy_cmd];
           proj_mot = (M\obj.acado_vals.MPC_output.x(:,1:2)' + [right_corner(1)*ones(obj.acado_vals.CH+1,1) right_corner(2)*ones(obj.acado_vals.CH+1,1)]')';
           obj.acado_vals.projected_motion.x = proj_mot(:,1);
           obj.acado_vals.projected_motion.y = proj_mot(:,2);
       end
       function outer_control(obj)
           obj.mpc_a2v();
       end
       function inner_control(obj)
           K_p_theta = 40;
           vx_cmd = obj.outer_cmd.vals(1);
           vy_cmd = obj.outer_cmd.vals(2);
           vec = [vx_cmd,vy_cmd];
           v_cmd = min(max(norm(vec),-obj.physical.v_max),obj.physical.v_max);
           e_theta = atan2(vec(2),vec(1)) - obj.position.theta;
           e_theta = atan2(sin(e_theta),cos(e_theta));
           omega_cmd = K_p_theta*e_theta;
           omega_cmd = min(max(omega_cmd,-obj.physical.omega_max),obj.physical.omega_max);
           obj.inner_cmd.vals = [v_cmd,omega_cmd];
       end
       function motion_step(obj)
           % Implements lower level controller and motion
           persistent vel_old;
           if isempty(vel_old)
              vel_old = [0.0, 0.0]; 
           end
           v_cmd = obj.inner_cmd.vals(1);
           omega_cmd = obj.inner_cmd.vals(2);
           [wL,wR] = inverseKinematics(obj.dd,v_cmd,omega_cmd);
           [v,w] = forwardKinematics(obj.dd,wL,wR);
           velB = [v;0;w]; % Body velocities [vx;vy;w]
           vel = bodyToWorld(velB,[obj.position.x;obj.position.y;obj.position.theta]);
           pose = [obj.position.x;obj.position.y;obj.position.theta] + vel*obj.sim_dt;
           acc = (vel - vel_old)/obj.sim_dt;
           vel_old = vel;
           obj.position.x = pose(1);
           obj.position.y = pose(2);
           obj.vel.x = vel(1);
           obj.vel.y = vel(2);
           obj.acc.x = acc(1);
           obj.acc.y = acc(2);
           obj.position.theta = atan2(sin(pose(3)),cos(pose(3)));
           obj.update_orientation();
       end
       function x_dot = EOM(obj,x)
           x_dot = zeros(6,1);
           x_dot(1) = x(3);
           x_dot(2) = x(4);
           x_dot(3) = x(5);
           x_dot(4) = x(6);
       end
       
   end
end