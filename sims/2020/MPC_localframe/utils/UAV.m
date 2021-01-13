classdef UAV < AMR
    properties
%        physical = struct(...
%            "mass",1.0,...
%            "force_max",1.0);
    end
    methods
        function obj = UAV(map)
           obj = obj@AMR(map);
           obj.update_orientation();
           obj.outer_cmd.names = ["ax","ay"];
           obj.outer_cmd.units = "Accleration (m/s^2)";
           obj.inner_cmd.names = ["thrust","roll","pitch","yaw"];
           A = [zeros(2) eye(2);...
               zeros(2,4)];
           B = [zeros(2);...
               eye(2)];
           C = [eye(2) zeros(2)];
           D = zeros(2);
           obj.process.model_continuous = ss(A,B,C,D);
           obj.process.model_discrete = c2d(obj.process.model_continuous,obj.filter_dt);
           obj.process.noise.var = 0.00000001*eye(4);
           obj.measurement.noise.var = 0.00000001*eye(size(C,1));
           init_est_state = [map.robot_suggest.pose_start.x;...
               map.robot_suggest.pose_start.y;...
               zeros(2,1)];
           obj.filter = StateFilter(obj.measurement,obj.process,obj.filter_dt,init_est_state);
           obj.process.state = init_est_state;
        end
        function mpc_acc(obj)
            pos_in = [obj.process.state(1) obj.process.state(2)];
            vel_in = [obj.process.state(3) obj.process.state(4)];
            wypt_in = [obj.MPC_vals.wypt.x obj.MPC_vals.wypt.y];
            occluded_corner_in = [obj.MPC_vals.occluded_corner.x,obj.MPC_vals.occluded_corner.y];
            M = obj.MPC_vals.localframe.M;
            localframe_origin = [obj.MPC_vals.localframe.x,obj.MPC_vals.localframe.y];
            % Transform
            [x,y] = c2u(pos_in(1),pos_in(2),localframe_origin(1),localframe_origin(2),M);
            [vx,vy] = c2u(vel_in(1),vel_in(2),0.0,0.0,M);
            [xg,yg] = c2u(wypt_in(1),wypt_in(2),localframe_origin(1),localframe_origin(2),M);
            [rc_x,rc_y] = c2u(occluded_corner_in(1),occluded_corner_in(2),localframe_origin(1),localframe_origin(2),M);
            % Slopes
            m_r = (y-rc_y)/(x-rc_x);
            if obj.MPC_vals.right_corner.active
                m_r_inv = (x-rc_x)/(y-rc_y);
            else
                m_r_inv = 0;
            end
            
            % NEW 11/24/2020
            % Really quick fix for angling position constraints
%             if obj.position.section==2 %&& x > 0
%                m_r_inv = -tan(80*pi/180); 
%             end
            
            phi_r = atan(m_r);
            phi_r_y = phi_r/y;
            phi_r_y_des = 0.0;
            % If not safe, use perception around corner
            %            obj.MPC_vals.safe = false;
            %            if obj.MPC_vals.safe || ~obj.MPC_vals.right_corner.active
            %                perc_r_weight = 0.01;
            %                y_offset = 10; % used to push position of corner "up" in MPC
            %                x_weight = 50;
            %            else
            %                perc_r_weight = 1000;
            %                y_offset = 0;
            %                x_weight = 10;
            %            end
            if obj.MPC_vals.corner_offset
                y_offset = 10;
            else
                y_offset = 0;
            end
            % Setup MPC values
%             left_m = 0;
%             left_x = -1.5;
%             left_y = y;
%             right_m = 0;
%             right_x = 1.5;
%             right_y = 0;
            left_m = cot(obj.MPC_vals.left_constraint.theta);
            left_x = obj.MPC_vals.left_constraint.x;
            left_y = obj.MPC_vals.left_constraint.y;
            right_m = cot(obj.MPC_vals.right_constraint.theta);
            right_x = obj.MPC_vals.right_constraint.x;
            right_y = obj.MPC_vals.right_constraint.y;
            top = obj.MPC_vals.top_constraint;
            [xrc,yrc] = c2u(obj.MPC_vals.occluded_corner.x,...
                   obj.MPC_vals.occluded_corner.y,...
                   obj.MPC_vals.localframe.x,...
                   obj.MPC_vals.localframe.y,...
                   obj.MPC_vals.localframe.M);
            corner_x = xrc;
            corner_y = yrc;
            phi_r = atan( (y-yrc-y_offset)/(x-xrc) )/(y-yrc-y_offset);
            
            obj.acado_vals.MPC_input.u = zeros(obj.acado_vals.CH,obj.acado_vals.input_num);
            obj.acado_vals.MPC_input.x0 = [x,y,vx,vy,phi_r,left_m,left_x,left_y,right_m,right_x,right_y,top,corner_x,corner_y,y_offset,0];
            obj.acado_vals.MPC_input.x = repmat(obj.acado_vals.MPC_input.x0,obj.acado_vals.CH+1,1);
            obj.acado_vals.MPC_input.y = repmat([xg,yg,0,0,0,0,0,0],obj.acado_vals.CH,1);
            obj.acado_vals.MPC_input.yN = [xg yg];
            % Set MPC weights
            % x, y, perception right, perception left, ax, ay, epsilon
            epsilon_weight = 1000000000000;
            %            epsilon_weight = 0.001;
            A = diag([obj.MPC_vals.weights.x,...
                obj.MPC_vals.weights.y,...
                obj.MPC_vals.weights.perc_r,...
                obj.MPC_vals.weights.vx,...
                obj.MPC_vals.weights.vy,...
                obj.MPC_vals.weights.cmd_x,...
                obj.MPC_vals.weights.cmd_y,...
                epsilon_weight]);
            obj.acado_vals.MPC_input.W = repmat(A,obj.acado_vals.CH,1);
            obj.acado_vals.MPC_input.WN = 100*diag([A(1,1) A(2,2)]);
            % Solve
            obj.acado_vals.MPC_output = acado_solver_acc_cmd_localframe( obj.acado_vals.MPC_input );
            as = obj.acado_vals.MPC_output.u(1,:);
            ax = as(1);
            ay = as(2);
            [ax_cmd,ay_cmd] = c2u(ax,ay,0,0,inv(M));
            obj.outer_cmd.vals = [ax_cmd,ay_cmd];
            proj_mot = (M\obj.acado_vals.MPC_output.x(:,1:2)' + [localframe_origin(1)*ones(obj.acado_vals.CH+1,1) localframe_origin(2)*ones(obj.acado_vals.CH+1,1)]')';
            obj.acado_vals.projected_motion.x = proj_mot(:,1);
            obj.acado_vals.projected_motion.y = proj_mot(:,2);
        end
        function set_MPC_weights(obj,map,params)
           % TODO: replace this hard-coded check with something better
%            if obj.position.section==1
%               obj.MPC_vals.safe = false; 
%            end
%            if obj.position.section==2
%               obj.MPC_vals.safe = true; 
%            end
           if obj.MPC_vals.right_corner.active && obj.position.section==1
               obj.MPC_vals.weights.x = params.weights.x;
               obj.MPC_vals.weights.y = params.weights.y;
               obj.MPC_vals.weights.vx = params.weights.vx;
               obj.MPC_vals.weights.vy = params.weights.vy;
%                obj.MPC_vals.weights.perc_r = params.weights.perc;
               obj.MPC_vals.weights.cmd_x = params.weights.cmd_x;
               obj.MPC_vals.weights.cmd_y = params.weights.cmd_y;
%                obj.MPC_vals.max_vel = params.max_vel;
%                obj.MPC_vals.corner_offset = params.corner_offset;
               if obj.safety.next_section || map.traffic_direction==0
                   obj.MPC_vals.corner_offset = 1;
                   obj.MPC_vals.weights.perc_r = 0.001;
               else
                   obj.MPC_vals.corner_offset = 0;
                   obj.MPC_vals.weights.perc_r = 1000;
               end
           else
               obj.MPC_vals.weights.perc_r = 0.01;
               obj.MPC_vals.weights.y_offset = 10; % used to push position of corner "up" in MPC
               obj.MPC_vals.weights.x_weight = 50;
               obj.MPC_vals.weights.y_weight = 50;
               obj.MPC_vals.weights.cmd_x = 25;
               obj.MPC_vals.weights.cmd_y = 25;
%                obj.MPC_vals.max_vel = 2.0;
               obj.MPC_vals.corner_offset = params.corner_offset;
           end
        end
        function outer_control(obj)
           obj.mpc_acc(); 
        end
        function inner_control(obj)
            % TODO: Replace inner control with trpy dynamics
            obj.inner_cmd.vals = obj.outer_cmd.vals;
        end
        function motion_step(obj)
            x0 = [obj.position.x;...
                obj.position.y;...
                obj.vel.x;...
                obj.vel.y];
            u = [obj.inner_cmd.vals(1),obj.inner_cmd.vals(2)];
            [t,x] = ode45(@(t,x) obj.EOM(t,x,u), [obj.time obj.time+obj.sim_dt], x0);
            noise = zeros(4,1);
            if obj.process.noise.enable
               var = obj.process.noise.var;
               noise = mvnrnd(zeros(size(obj.process.model_discrete.A,1),1),var)';
            end
            obj.position.x = x(end,1) + noise(1);
            obj.position.y = x(end,2) + noise(2);
            obj.vel.x = x(end,3) + noise(3);
            obj.vel.y = x(end,4) + noise(4);
            obj.acc.x = u(1);
            obj.acc.y = u(2);
            obj.process.state = [obj.position.x;...
                obj.position.y;...
                obj.vel.x;...
                obj.vel.y];
            obj.process.input = [obj.acc.x;...
                obj.acc.y];
            obj.update_orientation();
        end
        function xdot = EOM(obj,t,x,u)
            xdot = zeros(4,1);
            xdot(1) = x(3);
            xdot(2) = x(4);
            xdot(3) = u(1);
            xdot(4) = u(2);
        end
    end
end