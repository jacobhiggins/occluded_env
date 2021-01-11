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
           "max_vel",2.0,...
           "M",eye(2),...
           "wypt",struct("x",0.0,"y",0.0),...
           "ref_wypt",struct("x",0.0,"y",0.0),...
           "lims",struct("upper",100.0,"left",-5.0),...
           "safe",false);
       acado_vals = struct(...
           "CH",50,...
           "state_num",6,...
           "input_num",4,...
           "MPC_input",[],...
           "MPC_output",[],...
           "projected_motion",struct("x",[],"y",[]));
       stop_dist = 0.5;
       safe = false;
       outer_cmd = struct();
       inner_cmd = struct();
       lidar = LidarSensor();
       perception = struct("visible_area",struct("x",[],"y",[]),...
           "FOV",struct("x",[],"y",[]),...
           "occluded_points",struct("x",[],"y",[]),...
           "KU",struct("x",[],"y",[],"area",0.0));
       safety = struct("dist2coll",10,"collision_imminent",false);
       physical = struct("acc_max",2);
       dc;
       sim_dt;
       control_Hz = struct("outer",10,"inner",100); % LL control speed
       start = true;
   end
   methods
       function obj = AMR(map)
          obj.position.x = map.robot_suggest.pose_start.x;
          obj.position.y = map.robot_suggest.pose_start.y;
          obj.sim_dt = 1/obj.control_Hz.inner;
          obj.update_orientation();
          obj.lidar.measure(obj,map);
          obj.get_MPC_vals(map);
          obj.get_wypt(map);
          obj.dc = Data_collec(obj);
          obj.get_KU(map);
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
                  obj.get_MPCconstraints(map,i);
                  return;
              end
          end
       end
       function get_MPCconstraints(obj,map,region_num)
           
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
           
           [x_wypt,y_wypt,min_dist] = obj.intersection_wypt(map,current_ref_wypt);
           
           if isempty(y_wypt)
               current_ref_wypt = min(length(map.ref_traj.base_points.x)-1,current_ref_wypt+1);
               [x_wypt,y_wypt,min_dist] = obj.intersection_wypt(map,current_ref_wypt);
           end
           [x_wypt_tf,y_wypt_tf] = c2u(x_wypt,y_wypt,obj.position.x,obj.position.y,obj.MPC_vals.M);
           if y_wypt_tf < 0
               current_ref_wypt = min(length(map.ref_traj.base_points.x)-1,current_ref_wypt+1);
               [x_wypt,y_wypt,min_dist] = obj.intersection_wypt(map,current_ref_wypt);
           end
           obj.MPC_vals.wypt.x = x_wypt;
           obj.MPC_vals.wypt.y = y_wypt;
           

       end
       function [x_wypt,y_wypt,min_dist] = intersection_wypt(obj,map,current_ref_wypt)
           next_ref_wypt = current_ref_wypt + 1;
           % NEW 11/11/2020
           if (obj.position.section == 2) || (obj.position.section == 3)
              intersector = obj.perception.FOV; 
           else
              intersector = obj.perception.visible_area;
           end
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
       function logic_step(obj,map,weights)
          persistent last_outer_control;
          obj.set_MPC_weights(weights);
          obj.get_MPC_vals(map);
          obj.get_wypt(map);
          obj.get_KU(map);
          if  isempty(last_outer_control) || obj.start
             obj.outer_control();
             obj.expected_d2c(map);
             last_outer_control = obj.time;
          elseif ((obj.time-last_outer_control) >= 1/obj.control_Hz.outer)
             obj.outer_control();
             obj.expected_d2c(map);
             last_outer_control = obj.time;
          end
          obj.inner_control();
          obj.motion_step();
          obj.lidar.measure(obj,map);
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