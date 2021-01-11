classdef AMR < handle
   properties
       time = 0.0;
       position = struct("x",0.0,"y",0.0,"M",eye(2),"theta",pi/2,"section",1);
       vel = struct("x",0.0,"y",0.0);
       acc = struct("x",0.0,"y",0.0);
       MPC_vals = struct(...
           "right_corner",struct("x",0.0,"y",0.0,"active",false),...
           "left_corner",struct("x",0.0,"y",0.0),...
           "weights",struct("x",10,"y",10,"perc_r",0.01,"cmd_x",25,"cmd_y",25),...
           "localframe",struct("s",0.0,"x",0.0,"y",0.0,"M",eye(2)),...
           "M",eye(2),...
           "wypt",struct("x",0.0,"y",0.0,"occluded",false),...
           "ref_wypt",struct("x",0.0,"y",0.0),...
           "right_constraint",struct("x",1.5,"y",0.0,"theta",pi/2),...
           "right_const_global",struct("xs",1.5,"ys",0.0),...
           "left_constraint",struct("x",-1.5,"y",0.0,"theta",pi/2),...
           "left_const_global",struct("xs",-1.5,"ys",0.0),...
           "top_const_global",struct("xs",0,"ys",5.0),...
           "corner_offset",false);
       acado_vals = struct(...
           "CH",50,...
           "state_num",11,...
           "input_num",3,...
           "MPC_input",[],...
           "MPC_output",[],...
           "projected_motion",struct("x",[],"y",[]));
       stop_dist = 0.5;
       safe = false;
       outer_cmd = struct();
       inner_cmd = struct();
       lidar = LidarSensor();
       filter; % state filter for gaussian estimation
       perception = struct("visible_area",struct("x",[],"y",[]),...
           "FOV",struct("x",[],"y",[]),...
           "occluded_points",struct("x",[],"y",[]),...
           "KU",struct("enable",true,"x",[],"y",[],"area",0.0));
       safety = struct("dist2coll",10,"collision_imminent",false);
       physical = struct("acc_max",2);
       % Process variables
       % - Noise = process noise
       % - var = variance of gaussian noise, must be assigned to a particular
       % - EOM structure
       process = struct("model_continuous",ss(),...
           "model_discrete",ss(),...
           "state",[],...
           "input",[],...
           "noise",struct("enable",false,"var",[]));
       
       % Measurement variables
       measurement = struct("noise",struct("enable",false,"var",0.001*eye(2))); 
       
       % Data collector object
       dc;
       sim_dt;
       filter_dt;
       control_Hz = struct("outer",10,"inner",100); % LL control speed
       start = true;
   end
   methods
       function obj = AMR(map)
          obj.position.x = map.robot_suggest.pose_start.x;
          obj.position.y = map.robot_suggest.pose_start.y;
          obj.sim_dt = 1/obj.control_Hz.inner;
          obj.filter_dt = obj.sim_dt;
          obj.update_orientation();
          obj.lidar.measure(obj,map);
          obj.get_MPC_vals(map);
          obj.get_wypt(map);
          obj.dc = Data_collec(obj);
%           obj.get_KU(map); % SHOULD UNCOMMENT EVENTUALLY
       end
       function update_orientation(obj)
           obj.position.M = [cos(obj.position.theta) sin(obj.position.theta);...
               -sin(obj.position.theta) cos(obj.position.theta)];
       end
       function get_MPC_vals(obj,map)
          for i = 1:length(map.regions)
              region = map.regions{i};
              xlim = region.lims.x;
              ylim = region.lims.y;
              if inpolygon(obj.position.x,obj.position.y,xlim,ylim)
                  obj.position.section = i;
                  obj.MPC_vals.right_corner.x = region.corner(1);
                  obj.MPC_vals.right_corner.y = region.corner(2);
                  obj.MPC_vals.M = region.M;
                  % Try setting upper bound
                  try
                     obj.MPC_vals.lims.upper = region.lims.upper; 
                  catch
                     obj.MPC_vals.lims.upper = 100.0;
                  end
                  % Try setting left bound
                  try
                     obj.MPC_vals.lims.left = region.lims.left; 
                  catch
                     obj.MPC_vals.lims.left = -20;
                  end
                  
                  if norm([obj.position.x-region.corner(1),obj.position.y-region.corner(2)]) < obj.lidar.maxRange
                      obj.MPC_vals.right_corner.active = true;
                  else
                      obj.MPC_vals.right_corner.active = false; 
                  end
                  
%                   obj.get_s(region); % SHOULD UNCOMMENT LATER
                  obj.set_constraints(region);
                  
                  return;
              end
          end
       end
       function get_s(obj,region)
           % Get parameter s for generalized trajectory (assumed to be
           % line)
           % region.curl = +1 for clockwise motion
           % region.curl = -1 for counter-clockwise motion
           vector = region.localframe.wypt_vec;
           orth_vector = region.curl*[vector(2),-vector(1)];
           base.x = region.localframe.wypt_base.x;
           base.y = region.localframe.wypt_base.y;
           
           s = vector*[obj.position.x - base.x;obj.position.y - base.y];
           obj.MPC_vals.localframe.s = s;
           obj.MPC_vals.localframe.x = base.x + s*vector(1);
           obj.MPC_vals.localframe.y = base.y + s*vector(2);
           
           obj.MPC_vals.localframe.M = [orth_vector;vector];
       end
       
%        function get_s(obj,region)
%            % Get parameter s for generalized trajectory (assumed to be
%            % map_halfcircle)
%            % region.curl = +1 for clockwise motion
%            % region.curl = -1 for counter-clockwise motion
%            
%            base.x = region.base.x;
%            base.y = region.base.y;
%            
%            s = atan2(obj.position.y,base.x + region.r - obj.position.x);
%            vector = [sin(s),cos(s)];
%            orth_vector = [cos(s),-sin(s)];
%            
%            
%            obj.MPC_vals.localframe.s = s;
%            obj.MPC_vals.localframe.x = region.r*(1-cos(s)) + base.x;
%            obj.MPC_vals.localframe.y = region.r*sin(s) + base.y;
%            
%            obj.MPC_vals.localframe.M = [orth_vector;vector];
%        end
       
       function set_constraints(obj,region)
           xl = -region.width/2;
           yl = 0;
           thetal = pi/2;
           
           obj.MPC_vals.left_constraint.x = xl;
           obj.MPC_vals.left_constraint.y = yl;
           obj.MPC_vals.left_constraint.theta = thetal;
           obj.MPC_vals.top_constraint = region.top_constraint;
           
           if obj.MPC_vals.right_corner.active == true
               [xrc,yrc] = c2u(obj.MPC_vals.right_corner.x,...
                   obj.MPC_vals.right_corner.y,...
                   obj.MPC_vals.localframe.x,...
                   obj.MPC_vals.localframe.y,...
                   obj.MPC_vals.localframe.M);
               [x,y] = c2u(obj.position.x,...
                   obj.position.y,...
                   obj.MPC_vals.localframe.x,...
                   obj.MPC_vals.localframe.y,...
                   obj.MPC_vals.localframe.M);
%                thetar = atan2(yrc-y,xrc-x);
               thetar = pi/2; % CHANGE BACK ******
                   
               obj.MPC_vals.right_constraint.x = xrc;
               obj.MPC_vals.right_constraint.y = yrc;
               obj.MPC_vals.right_constraint.theta = thetar;
           else
               xrc = region.width/2;
               yrc = 0;
               thetar = pi/2;
               obj.MPC_vals.right_constraint.x = xrc;
               obj.MPC_vals.right_constraint.y = yrc;
               obj.MPC_vals.right_constraint.theta = thetar;
           end
           rs = [50,0,-50];
           xs_left = rs*cos(thetal) + xl;
           ys_left = rs*sin(thetal) + yl;
           xs_right = rs*cos(thetar) + xrc;
           ys_right = rs*sin(thetar) + yrc;
           xs_top = [-50,0,50];
           ys_top = obj.MPC_vals.top_constraint*ones(1,3); % Hard coded, change!
           % PARABOLIC CONSTRAINTS
%            ys_right = -20:0.1:20;
%            xs_right = 1*ys_right.^2 + xrc;
           [xs_left,ys_left] = u2c(xs_left,ys_left,...
               obj.MPC_vals.localframe.x,obj.MPC_vals.localframe.y,obj.MPC_vals.localframe.M);
           [xs_right,ys_right] = u2c(xs_right,ys_right,...
               obj.MPC_vals.localframe.x,obj.MPC_vals.localframe.y,obj.MPC_vals.localframe.M);
           [xs_top,ys_top] = u2c(xs_top,ys_top,...
               obj.MPC_vals.localframe.x,obj.MPC_vals.localframe.y,obj.MPC_vals.localframe.M);
           obj.MPC_vals.left_const_global.xs = xs_left;
           obj.MPC_vals.left_const_global.ys = ys_left;
           obj.MPC_vals.right_const_global.xs = xs_right;
           obj.MPC_vals.right_const_global.ys = ys_right;
           obj.MPC_vals.top_const_global.xs = xs_top;
           obj.MPC_vals.top_const_global.ys = ys_top;
       end
       
       function get_cornerWP(obj,map)
           
       end
       function get_wypt(obj,map)
           persistent current_ref_wypt;
           if isempty(current_ref_wypt) || obj.start
              current_ref_wypt = 1; 
           end
           obj.MPC_vals.ref_wypt.x = map.ref_traj.base_points.x(current_ref_wypt+1);
           obj.MPC_vals.ref_wypt.y = map.ref_traj.base_points.y(current_ref_wypt+1);
           
           intersector = obj.perception.FOV;
           [x_wypt,y_wypt,min_dist] = obj.intersection_wypt(map,current_ref_wypt,intersector);
           
           % If wypt empty, iterate through other wypt base pairs until
           % intersection is found (up until last pair)
           if isempty(y_wypt) && (current_ref_wypt < length(map.ref_traj.base_points.x)-1)
               while current_ref_wypt < length(map.ref_traj.base_points.x)-1
                   current_ref_wypt = min(length(map.ref_traj.base_points.x)-1,current_ref_wypt+1);
                   [x_wypt,y_wypt,min_dist] = obj.intersection_wypt(map,current_ref_wypt,intersector);
                   if ~isempty(y_wypt)
                      break; 
                   end
               end
           end
           
           % If y_wypt still empty, then that means we've reach end of traj
           % Use last waypoint base
           if isempty(y_wypt)
               obj.MPC_vals.wypt.x = map.ref_traj.base_points.x(end);
               obj.MPC_vals.wypt.y = map.ref_traj.base_points.y(end);
               return;
           end
           
           % This moves wypt from edge-of-FOV to where visible, open region
           % intersects with reference trajectory
           % Determine if waypoint is occluded
%            intersector = obj.perception.visible_area;
%            [x_vis,y_vis,min_dist] = obj.intersection_wypt(map,current_ref_wypt,intersector);
%            if isempty(y_vis) || norm([x_vis-x_wypt,y_vis-y_wypt])
%                % This means waypoint is occluded
%                obj.MPC_vals.occluded = true;
%                free_avg = [obj.lidar.freespace.x(1,:)+obj.lidar.freespace.x(2,:);... % average between all free-space point pairs
%                    obj.lidar.freespace.y(1,:)+obj.lidar.freespace.y(2,:)]/2;
%                diffs = [free_avg(1,:) - x_wypt; free_avg(2,:) - y_wypt];
%                rs = vecnorm(diffs);
%                [~,I] = min(rs);
%                x_wypt = free_avg(1,I);
%                y_wypt = free_avg(2,I);
%            else
%                
%            end
           
           obj.MPC_vals.wypt.x = x_wypt;
           obj.MPC_vals.wypt.y = y_wypt;
           

       end
       function [x_wypt,y_wypt,min_dist] = intersection_wypt(obj,map,current_ref_wypt,intersector)
           next_ref_wypt = current_ref_wypt + 1;
           [xi,yi] = polyxpoly(map.ref_traj.base_points.x(current_ref_wypt:next_ref_wypt),...
               map.ref_traj.base_points.y(current_ref_wypt:next_ref_wypt),...
               intersector.x,...
               intersector.y);
           
           [min_dist,I] = min((xi-obj.MPC_vals.ref_wypt.x).^2 + (yi-obj.MPC_vals.ref_wypt.y).^2);
           x_wypt = xi(I);
           y_wypt = yi(I);
       end
       function shift_wypt(obj,map)
           
       end
       function expected_d2c(obj,map)
           index = length(obj.acado_vals.projected_motion.x);
           collision_iminent = false;
           for i = 1:length(map.walls)
               wall = map.walls{i};
               in = inpolygon(obj.acado_vals.projected_motion.x,obj.acado_vals.projected_motion.y,wall.xs,wall.ys+0.01);
               index_maybe = find(in,1);
               % Only care about obstacle in hallway, i==4
               if ~isempty(index_maybe) && i==4
                   index = min(index,index_maybe);
                   collision_iminent = true;
               end
           end
           obj.safety.collision_imminent = collision_iminent;
           if collision_iminent
%                points_before_coll = [obj.acado_vals.projected_motion.x(1:index),obj.acado_vals.projected_motion.y(1:index)];
%                dist = sum(vecnorm([ points_before_coll(1:end-1,1) - points_before_coll(2:end,1) , points_before_coll(1:end-1,2) - points_before_coll(2:end,2) ],2,2));
%                obj.MPC_vals.max_vel = min(sqrt(2*obj.physical.acc_max*dist),2);
%                 obj.MPC_vals.max_vel = 0.5;
           else
              obj.MPC_vals.max_vel = 2; 
           end
       end
       function get_KU(obj,map)
           if ~obj.perception.KU.enable
              obj.perception.occluded_points.x = obj.position.x*ones(1,4);
              obj.perception.occluded_points.y = obj.position.y*ones(1,4);
              obj.perception.KU.x = obj.position.x*ones(1,4);
              obj.perception.KU.y = obj.position.y*ones(1,4);
              obj.perception.KU.area = 0.0;
              return; 
           end
           
           points_in_map_Is = inpolygon(obj.perception.FOV.x,obj.perception.FOV.y,map.total_region.xs,map.total_region.ys);
           point_short_range_Is= obj.lidar.ranges < obj.lidar.maxRange;
           union_Is = logical(points_in_map_Is.*point_short_range_Is);
           obj.perception.occluded_points.x = obj.perception.FOV.x(union_Is);
           obj.perception.occluded_points.y = obj.perception.FOV.y(union_Is);
           if isempty(obj.perception.occluded_points.x)
              obj.perception.occluded_points.x = obj.position.x*ones(1,4);
              obj.perception.occluded_points.y = obj.position.y*ones(1,4);
              obj.perception.KU.x = obj.position.x*ones(1,4);
              obj.perception.KU.y = obj.position.y*ones(1,4);
              obj.perception.KU.area = 0.0;
              return
           end
           [points_x,points_y] = c2u(obj.perception.occluded_points.x,...
               obj.perception.occluded_points.y,...
               obj.MPC_vals.right_corner.x,...
               obj.MPC_vals.right_corner.y,...
               obj.MPC_vals.M);
           obj.perception.KU.x = [obj.perception.occluded_points.x(points_y>0) obj.MPC_vals.right_corner.x];
           obj.perception.KU.y = [obj.perception.occluded_points.y(points_y>0) obj.MPC_vals.right_corner.y];
           obj.perception.KU.area = polyarea(obj.perception.KU.x,obj.perception.KU.y);
       end
       function get_state(obj)
           obj.process.state = obj.filter.estimate(obj);
       end
       
       function get_KU_fast(obj,map)
           if obj.position.section==2
               
           end
       end
       
       function update_occ(obj)
           
       end
       function set_MPC_weigts(obj) end;
       function inner_control(obj) end;
       function outer_control(obj)
          obj.go2goal(); 
       end
       function motion_step(obj) end;
       function go2goal(obj)
           xr = obj.position.x;
           yr = obj.position.y;
           xw = obj.MPC_vals.wypt.x;
           yw = obj.MPC_vals.wypt.y;
           
           x_cmd = xr - xw;
           y_cmd = yr - yw;
           
           obj.outer_cmd.vals = [x_cmd,y_cmd];
       end
       function logic_step(obj,map,params)
          persistent last_outer_control;
          obj.set_MPC_weights(params);
          obj.get_MPC_vals(map);
%           obj.get_KU(map); % SHOULD UNCOMMENT EVENTUALLY
          obj.get_state();
          if  isempty(last_outer_control) || obj.start
              obj.lidar.measure(obj,map);
              obj.get_wypt(map);
             obj.outer_control();
             obj.expected_d2c(map);
             last_outer_control = obj.time;
          elseif ((obj.time-last_outer_control) >= 1/obj.control_Hz.outer)
              obj.lidar.measure(obj,map);
              obj.get_wypt(map);
             obj.outer_control();
             obj.expected_d2c(map);
             last_outer_control = obj.time;
          end
          obj.inner_control();
          obj.motion_step();
          
          obj.get_wypt(map); % To plot correctly
          if obj.dc.collect_on
             obj.dc.record(obj); 
          end
          if obj.dc.collect_stats
             obj.dc.record_stats(obj,map); 
          end
          obj.time = obj.time + obj.sim_dt;
          obj.start = false;
       end
   end
end