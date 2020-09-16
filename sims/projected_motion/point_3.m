classdef point_3 < point_2
   properties
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
           obj.x = map.pose_start.x;
           obj.y = map.pose_start.y;
           obj.get_wypt(map);
           obj.M_mpc = eye(2);
           obj.MPCinput.u = zeros(obj.N,obj.Nu);
       end
       function get_wypt(obj,map)
          persistent i;
          if isempty(i)
             i = 1; 
          end
          if isempty(obj.wypt)
              wypt = map.wypts{i};
          else
              wypt = obj.wypt;
          end
          if norm([wypt.x-obj.x;wypt.y-obj.y]) < 0.1
              i = mod(i,length(map.wypts))+1;
          end
          obj.wypt = map.wypts{i};
       end
       function get_wypt_pursuit(obj,map)
          if obj.wypt.y >= 10
             obj.wypt.y = 10;
             obj.wypt.x = 5;
          else
             obj.wypt.y = obj.y + 5;
             obj.wypt.x = obj.x ;
          end
       end
       function mpc_step3(obj)
           % Use both left and right corners instead of just the right
           left_bound = obj.current_owall;
           right_bound = 0;
           if obj.current_sec==3
               right_bound = 15;
           end
           xr_in = obj.x;
           yr_in = obj.y;
           xc_r_in = 10;
           yc_r_in = 10;
           xc_l_in = -10;
           yc_l_in = 10;
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
           
           m_r_inv = 0;
           m_l_inv = 0;
           
           if obj.lc_active
              perc_l_weight = 5; 
           else
              perc_l_weight = 5;
           end
           
           if obj.rc_active
              perc_r_weight = 0.5; %10000000; 
           else
              perc_r_weight = 0.5;
           end
           
           var_r = phi_r_y;
           var_l = phi_l_y;
           var_des = 0;
           
           obj.MPCinput.x0 = [xr,yr,vx,vy,ax,ay,var_r,var_l,xc_l,yc_l,left_bound,right_bound,m_l_inv,m_r_inv,0];
           
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
               m_l_inv*ones(obj.N+1,1) ...
               m_r_inv*ones(obj.N+1,1) ...
               0*ones(obj.N+1,1)];
           
           obj.MPCinput.y = [xg*ones(obj.N,1), yg*ones(obj.N,1), var_r*ones(obj.N,1), var_l*ones(obj.N,1), 0*ones(obj.N,1),0*ones(obj.N,1),0*ones(obj.N,1)];
           obj.MPCinput.yN = [xg yg var_des var_des];
           
           % x, y, perception right, perception left, ax_dot, ay_dot, epsilon
           A = diag([50 50 perc_r_weight perc_l_weight 10 10 5]);
           
%            if obj.current_sec==3
%                A(3,3) = 0.000000000001;
%            end
           
           obj.MPCinput.W = repmat(A,obj.N,1);
%            A_total = [];
%            for i= 1:obj.N
%               A_total = [A_total;diag([A(1,1),A(2,2)^(1+i*0.001),A(3,3),A(4,4),A(5,5),A(6,6),A(7,7)])]; 
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
   end
end