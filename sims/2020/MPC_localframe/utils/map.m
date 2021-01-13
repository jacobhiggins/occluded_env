classdef map < handle
   properties
       robot_suggest = struct(...
           "maxRad",10.0,...
           "pose_start",struct("x",0.0,"y",0.0)...
       )
       dims = struct(...
           "width",5.0,...
           "height",5.0,...
           "resolution",10, ...
           "dx",1, ...
           "wall_thickness",1, ...
           "xlim",[], ...
           "ylim", [] ...
       )
       figs = struct("main",[],"aux",[]);
       occ_map;
       traffic = {};
       ref_traj = struct("base_points",struct("x",[],"y",[]),...
           "complete_points",struct("x",[],"y",[]));
       plts = containers.Map;
       plts_enable = struct("constraints",true,...
           "freespace",true,...
           "lidar",true);
       walls = {};
       epsilon = 0.01;
       vid = struct("rec",true,"writer",[]);
       regions = {};
       total_region = struct("xs",[],"ys",[],"polygon",0.0);
       end_flag = false;
       update_Hz = 10;
       heading_magnitude = 1;
   end
   methods
       function obj = map()
          obj.dims.dx = 1/obj.dims.resolution;
          obj.dims.wall_thickness = obj.dims.dx;
          if obj.vid.rec
             obj.vid.writer = VideoWriter("vid.mp4","MPEG-4");
             obj.vid.writer.FrameRate = 10;
             open(obj.vid.writer);
          end
       end
       function init_occ_map(obj)

          % Create occupancy map
          obj.occ_map = occupancyMap(obj.dims.width,obj.dims.height,obj.dims.resolution);
          
          % Set initial occupancy values
          for y = obj.dims.ylim(1)+obj.epsilon:obj.dims.dx:obj.dims.ylim(2)
             for x = obj.dims.xlim(1)+obj.epsilon:obj.dims.dx:obj.dims.xlim(2)
                occupied = false;
                trafficked = false;
                % Determine grid values that are occupied
                for i = 1:length(obj.walls)
                    if inpolygon(x,y,obj.walls{i}.xs,obj.walls{i}.ys)
                        occupied = true;
                    end
                end
                % Set occupancy value
                if occupied
                    setOccupancy(obj.occ_map,[x y],1);
                else
                    setOccupancy(obj.occ_map,[x y],0);
                end
             end
          end
          
       end
       function init_plot(obj,robot)
          fig_main = figure(1);
          obj.figs.main = fig_main;
          hold on;
          obj.init_occ_map();

          % Plot occupancy map
          occ_map_image = show(obj.occ_map);
          obj.plts("plt_occ_map") = occ_map_image;
          
          % Plot obstacles
          for i = 1:length(obj.walls)
              wall= obj.walls{i};
              black = zeros(1,3);
              fill(wall.xs,wall.ys,black);
          end
          
          % Plot traffic occupancy map
          for i = 1:length(obj.traffic)
              traffic_region = obj.traffic{i};
              plt_patches{traffic_region.width.x,traffic_region.width.y} = [];
              for xi = 1:size(traffic_region.X,2)
                  x = traffic_region.X(1,xi);
                  xs = traffic_region.delx*[-0.5,0.5] + x;
                  for yi = 1:size(traffic_region.Y,1)
                      y = traffic_region.Y(yi,1);
                      ys = traffic_region.dely*[-1,1] + y;
                      c = 1-traffic_region.occ.current(yi,xi);
                      plt_patches{xi,yi} = patch(xs([1 2 2 1]),ys([1 1 2 2]),c*ones(1,3),"EdgeColor","none");
                  end
              end
              obj.plts("plt_patches") = plt_patches;
          end
          
          %Plot FOV
          plt_FOV_area = plot(robot.perception.FOV.x,robot.perception.FOV.y,"g--");
          obj.plts("plt_FOV_area") = plt_FOV_area;

          % Plot lidar points
          if obj.plts_enable.lidar
              plt_lidar = scatter(robot.lidar.endpoints.x,robot.lidar.endpoints.y,...
                  8,...
                  'r',...
                  'filled');
              obj.plts("plt_lidar") = plt_lidar;
          end
          
          % Plot visible area
          blue = [173, 216, 230]/255;
          plt_perc_area = fill(robot.perception.visible_area.x,robot.perception.visible_area.y,blue,"FaceAlpha",0.3);
          obj.plts("plt_perc_area") = plt_perc_area;
          
          % Plot occluded area
          red = [255 204 204]/255;
          if robot.perception.KU.enable
              %           plt_occluded_area = scatter(robot.perception.KU.x,robot.perception.KU.y);
              plt_KU_area = plot(robot.perception.KU.poly);
              plt_KU_area.FaceAlpha = 0.3;
              plt_KU_area.FaceColor = red;
              %           plt_KU_area = fill(robot.perception.KU.x,robot.perception.KU.y,red);
              %           obj.plts("plt_occluded_area") = plt_occluded_area;
              obj.plts("plt_KU_area") = plt_KU_area;
          end
          
          % Plot reference trajectory
          plt_ref_traj = plot(obj.ref_traj.base_points.x,obj.ref_traj.base_points.y,'r--');
          obj.plts("plt_ref_traj") = plt_ref_traj;
          
          % Plot free space
          if obj.plts_enable.freespace
              plt_freespace = plot(robot.lidar.freespace.plot.x,...
                  robot.lidar.freespace.plot.y,...
                  'c-.',...
                  "LineWidth",...
                  2);
              obj.plts("plt_freespace") = plt_freespace;
          end
          
          % Plot waypoint
          plt_wypt = plot(robot.MPC_vals.wypt.x,robot.MPC_vals.wypt.y,"mx","MarkerSize",10);
          obj.plts("plt_wypt") = plt_wypt;
          
          % Plot projected motion
          robot.acado_vals.projected_motion.x = robot.position.x;
          robot.acado_vals.projected_motion.y = robot.position.y;
          plt_proj_mot = plot(robot.acado_vals.projected_motion.x,robot.acado_vals.projected_motion.y,...
              "LineWidth",2,...
              "Color","magenta");
          obj.plts("plt_proj_mot") = plt_proj_mot;
          
          % Plot actual trajectory
          plt_actual_mot = plot(robot.dc.positions.x,robot.dc.positions.y,...
              "b--",...
              "LineWidth",2);
          obj.plts("plt_actual_mot") = plt_actual_mot;
          
          % Plot left/right/top constaints
          if obj.plts_enable.constraints
              plt_left_constraint = plot(robot.MPC_vals.left_const_global.xs,...
                  robot.MPC_vals.left_const_global.ys,...
                  "c--",...
                  "LineWidth",2);
              obj.plts("plt_left_constraint") = plt_left_constraint;
              plt_right_constraint = plot(robot.MPC_vals.right_const_global.xs,...
                  robot.MPC_vals.right_const_global.ys,...
                  "r--",...
                  "LineWidth",2);
              obj.plts("plt_right_constraint") = plt_right_constraint;
              plt_top_constraint = plot(robot.MPC_vals.top_const_global.xs,...
                  robot.MPC_vals.top_const_global.ys,...
                  "g--",...
                  "LineWidth",2);
              obj.plts("plt_top_constraint") = plt_top_constraint;
          end
          
          % Plot robot
          plt_heading = quiver(robot.position.x,...
              robot.position.y,...
              obj.heading_magnitude*cos(robot.position.theta),...
              obj.heading_magnitude*sin(robot.position.theta));
          plt_robot = plot(robot.position.x,robot.position.y,'bo');
          obj.plts("plt_heading") = plt_heading;
          obj.plts("plt_robot") = plt_robot;
%           xlim(obj.dims.xlim);
%           ylim(obj.dims.ylim);
          if obj.vid.rec
              frame = getframe(gcf);
              writeVideo(obj.vid.writer,frame);
          end
       end
       
       function init_aux(obj,robot)
           figure(2);
           hold on;
           % Plot total velocity
           subplot(3,1,1);
           plt_vels_norm = plot(robot.dc.times,robot.dc.vels.norm,...
               "LineWidth",2);
           ylabel("Norm Velocity (m/s)");
           grid on;
           obj.plts("plt_vels_norm") = plt_vels_norm;
           % Plot command inputs
           subplot(3,1,2);
           hold on;
           plt_cmd1 = plot(robot.dc.times,robot.dc.cmds(:,1),...
               "LineWidth",2,...
               "DisplayName",robot.outer_cmd.names(1));
           plt_cmd2 = plot(robot.dc.times,robot.dc.cmds(:,2),...
               "LineWidth",2,...
               "DisplayName",robot.outer_cmd.names(2));
           grid on;
           legend([plt_cmd1,plt_cmd2],"Location","west");
           ylabel(robot.outer_cmd.units);
           obj.plts("plt_cmd1") = plt_cmd1;
           obj.plts("plt_cmd2") = plt_cmd2;
           % Plot KU area
           subplot(3,1,3);
           plt_KUs = plot(robot.dc.times,robot.dc.KUs.area,...
               "LineWidth",2);
           ylabel("KU area (m^2)");
           xlabel("Time (s)");
           grid on;
           obj.plts("plt_KUs") = plt_KUs;
           figure(1);
       end
       
       function update_plot(obj,robot)
           persistent last_update_time;
           if isempty(last_update_time) || robot.time < 0.1
              last_update_time = 0.0; 
           end
           if (robot.time-last_update_time) < 1/obj.update_Hz
               return;
           end
           title(sprintf("Time: %0.1f",robot.time));
           last_update_time = robot.time;
           % Update occupancy map
           occ_map_image = obj.plts("plt_occ_map");
           occ_map_image.CData = repmat(1 - occupancyMatrix(obj.occ_map),[1 1 3]);
           
           % Update lidar
            if obj.plts_enable.lidar
                plt_lidar = obj.plts("plt_lidar");
                plt_lidar.XData = robot.lidar.endpoints.x;
                plt_lidar.YData = robot.lidar.endpoints.y;
            end
           
           % Update occupancy
          for i = 1:length(obj.traffic)
              traffic_region = obj.traffic{i};
              plt_patches = obj.plts("plt_patches");
              for xi = 1:size(traffic_region.X,2)
                  x = traffic_region.X(1,xi);
                  for yi = 1:size(traffic_region.Y,1)
                      y = traffic_region.Y(yi,1);
                      c = 1-traffic_region.occ.future(yi,xi);
                      plt_patches{xi,yi}.FaceColor = c*ones(1,3);
                  end
              end
              obj.plts("plt_patches") = plt_patches;
          end
           
           % Update FOV
           plt_FOV_area = obj.plts("plt_FOV_area");
           plt_FOV_area.XData = robot.perception.FOV.x;
           plt_FOV_area.YData = robot.perception.FOV.y;
           
           % Update visible area
           plt_perc_area = obj.plts("plt_perc_area");
           plt_perc_area.XData = robot.perception.visible_area.x;
           plt_perc_area.YData = robot.perception.visible_area.y;
           
           % Update occluded points + KU area
%            plt_occluded_area = obj.plts("plt_occluded_area");
%            plt_occluded_area.XData = robot.perception.KU.x;
%            plt_occluded_area.YData = robot.perception.KU.y;
            if robot.perception.KU.enable
             plt_KU_area = obj.plts("plt_KU_area");
             plt_KU_area.Shape = robot.perception.KU.poly;
            end
           
           % Update freespace
           if obj.plts_enable.freespace
               plt_freespace = obj.plts("plt_freespace");
               plt_freespace.XData = robot.lidar.freespace.plot.x;
               plt_freespace.YData = robot.lidar.freespace.plot.y;
           end
           
           % Update waypoint
           plt_wypt = obj.plts("plt_wypt");
           plt_wypt.XData = robot.MPC_vals.wypt.x;
           plt_wypt.YData = robot.MPC_vals.wypt.y;
           
           % Updated projected motion
           plt_proj_mot = obj.plts("plt_proj_mot");
           plt_proj_mot.XData = robot.acado_vals.projected_motion.x;
           plt_proj_mot.YData = robot.acado_vals.projected_motion.y;
           
           plt_actual_mot = obj.plts("plt_actual_mot");
           plt_actual_mot.XData = robot.dc.positions.x;
           plt_actual_mot.YData = robot.dc.positions.y;
           
           % Update left/right/top constraint
           if obj.plts_enable.constraints
               plt_left_constraint = obj.plts("plt_left_constraint");
               plt_left_constraint.XData = robot.MPC_vals.left_const_global.xs;
               plt_left_constraint.YData = robot.MPC_vals.left_const_global.ys;
               plt_right_constraint = obj.plts("plt_right_constraint");
               plt_right_constraint.XData = robot.MPC_vals.right_const_global.xs;
               plt_right_constraint.YData = robot.MPC_vals.right_const_global.ys;
               plt_top_constraint = obj.plts("plt_top_constraint");
               plt_top_constraint.XData = robot.MPC_vals.top_const_global.xs;
               plt_top_constraint.YData = robot.MPC_vals.top_const_global.ys;
           end
           
           % Update robot
           plt_heading = obj.plts("plt_heading");
           plt_heading.XData = robot.position.x;
           plt_heading.YData = robot.position.y;
           plt_heading.UData = obj.heading_magnitude*cos(robot.position.theta);
           plt_heading.VData = obj.heading_magnitude*sin(robot.position.theta);
           plt_robot = obj.plts("plt_robot");
           plt_robot.XData = robot.position.x;
           plt_robot.YData = robot.position.y;
           
           if obj.vid.rec
              frame = getframe(gcf);
              writeVideo(obj.vid.writer,frame);
           end
       end
       function update_aux(obj,robot)
           persistent last_update_time;
           if isempty(last_update_time)
              last_update_time = 0.0; 
           end
           if (robot.time-last_update_time) < 1/obj.update_Hz
               return;
           end
           last_update_time = robot.time;
            plt_vels_norm = obj.plts("plt_vels_norm");
            plt_vels_norm.XData = robot.dc.times;
            plt_vels_norm.YData = robot.dc.vels.norm;
            plt_cmd1 = obj.plts("plt_cmd1");
            plt_cmd1.XData = robot.dc.times;
            plt_cmd1.YData = robot.dc.cmds(:,1);
            plt_cmd2 = obj.plts("plt_cmd2");
            plt_cmd2.XData = robot.dc.times;
            plt_cmd2.YData = robot.dc.cmds(:,2);
            plt_KUs = obj.plts("plt_KUs");
            plt_KUs.XData = robot.dc.times;
            plt_KUs.YData = robot.dc.KUs.area;
            
            drawnow();
       end
       function update_occ_map(obj,robot)
           persistent last_update_time;
           if isempty(last_update_time)
              last_update_time = 0.0; 
           end
           if (robot.time-last_update_time) < 1/obj.update_Hz
               return;
           end
           last_update_time = robot.time;
           traffic_region = obj.traffic{1};
           for xi = 1:size(traffic_region.X,2)
               x = traffic_region.X(1,xi);
               for yi = 1:size(traffic_region.Y,1)
                   y = traffic_region.Y(yi,1);
                   if InPolygon(x,y,robot.perception.visible_area.x,robot.perception.visible_area.y)
                       obj.traffic{1}.occ.current(yi,xi) = 0.0;
                   end
               end
           end
           obj.traffic{1}.occ.future = obj.traffic{1}.occ.current;
           obj.get_future_occ_map(robot);
       end
       function get_future_occ_map(obj,robot)
           traffic_region = obj.traffic{1};
           T = obj.get_future_time(robot);
           old_future_occ = traffic_region.occ.future;
           new_future_occ = traffic_region.occ.future;
           updated_map = false;
           for t = 0:1/(obj.update_Hz):T
               for xi = 1:size(traffic_region.X,2)
                   for yi = 1:size(traffic_region.Y,1)
                       if traffic_region.source_grid(yi,xi)
                           continue
                       end
                       y_indices = yi + traffic_region.motion_indices;
                       y_indices = max(y_indices,1);
                       y_indices = min(y_indices,size(traffic_region.occ.future,1));
                       % Find new future occupancy for single step
                       % Note: since increase y index means decreasing y
                       % value, must flip the motion model matrix
                       new_future_occ(yi,xi) = sum(old_future_occ(y_indices,xi).*flip(traffic_region.motion_model));
                   end
               end
               if ~updated_map
                   obj.traffic{1}.occ.current = new_future_occ;
                   updated_map = true;
               end
               old_future_occ = new_future_occ;
           end
           obj.traffic{1}.occ.future = new_future_occ;
       end
       function prob = get_grid_occupancy(obj,point)
           % Get occupancy value of a single grid square
           x_lim = obj.traffic{1}.lims.x;
           y_lim = obj.traffic{1}.lims.y;
           if ~InPolygon(point.x,point.y,x_lim([1 2 2 1 1]),y_lim([1 1 2 2 1]))
               % If point isn't in trafficked, section, return 0.0 (no
               % occupancy)
               prob = 0.0;
               return;  
           end
           % Find grid point closest to input point
           traffic_region = obj.traffic{1};
           dists = (traffic_region.X-point.x).^2 + (traffic_region.Y-point.y).^2;
           [~,I] = min(dists(:));
%            closest.X = traffic_region.X(I);
%            closest.Y = traffic_region.Y(I);
           prob = obj.traffic{1}.occ.future(I);
       end
       function close_map(obj)
          close(obj.vid.writer);
          close(obj.figs.main);
       end
   end
end