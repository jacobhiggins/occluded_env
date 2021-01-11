classdef map_1corner < map
   properties
      hall_dims = struct("w",[3,0.2],"l",[10.0,15]);
      with_obs = false;
   end
   methods
       function obj = map_1corner()
           obj = obj@map();
           obj.dims.width = obj.hall_dims.l(2) + obj.dims.wall_thickness;
           obj.dims.height = obj.hall_dims.l(1) + obj.dims.wall_thickness;
           obj.dims.resolution = 10;
           obj.dims.dx = 1/obj.dims.resolution;
           obj.dims.wall_thickness = obj.dims.dx;
           
           obj.robot_suggest.pose_start.x = obj.hall_dims.w(1)/2 + obj.dims.wall_thickness;
           obj.robot_suggest.pose_start.y = 0;
           
           wall = struct("xs",[],"ys",[]);
           % Left wall
           wall.xs = [0 obj.dims.wall_thickness*ones(1,2) 0 0];
           wall.ys = [zeros(1,2) (obj.hall_dims.l(1)+obj.dims.dx)*ones(1,2) 0];
           obj.walls{end+1} = wall;
           % Top wall
           wall.xs = [0 (obj.hall_dims.l(2)+obj.dims.wall_thickness)*ones(1,2) 0 0];
           wall.ys = [obj.hall_dims.l(1)*ones(1,2) (obj.hall_dims.l(1)+obj.dims.dx)*ones(1,2) obj.hall_dims.l(1)];
           obj.walls{end+1} = wall;
           % Bottom right wall
           wall.xs = [obj.hall_dims.w(1) obj.hall_dims.l(2)*ones(1,2) obj.hall_dims.w(1)*ones(1,2)]+obj.dims.wall_thickness;
           wall.ys = [zeros(1,2) (obj.hall_dims.l(1)-obj.hall_dims.w(2))*ones(1,2) 0];
           obj.walls{end+1} = wall;
           % Define obstacle
           if obj.with_obs
              obj_length_points = [0.2,0.8];
              obj_width_points = [0.65,0.35];
              wallxs = obj.hall_dims.w(1) + (obj.hall_dims.l(2)-obj.hall_dims.w(1))*obj_length_points;
              wall.xs = wallxs([1 2 2 1 1]);
              wallys = obj.hall_dims.l(1) - obj.hall_dims.w(2)*obj_width_points;
              wall.ys = wallys([1 1 2 2 1]);
              obj.walls{end+1}=wall;
           end
           
           % Limits, for plotting
           obj.dims.xlim = [0 obj.hall_dims.l(2)+obj.dims.wall_thickness];
           obj.dims.ylim = [0 (obj.hall_dims.l(1)+obj.dims.dx)];
           
           % Reference trajectory definition
           obj.ref_traj.base_points.x = [obj.robot_suggest.pose_start.x,...
               obj.robot_suggest.pose_start.x,...
               100*obj.hall_dims.l(2)];
           obj.ref_traj.base_points.y = [obj.robot_suggest.pose_start.y,...
               obj.hall_dims.l(1)-obj.hall_dims.w(2)/2,...
               obj.hall_dims.l(1)-obj.hall_dims.w(2)/2];
           
           % Define regions for MPC
           
           xlim = [obj.dims.wall_thickness obj.hall_dims.w(1)];
           ylim = [0 obj.hall_dims.l(1)-obj.hall_dims.w(2)];
           region1.lims.x = xlim([1 1 2 2 1]);
           region1.lims.y = ylim([1 2 2 1 1]);
           region1.corner = [obj.hall_dims.w(1), obj.hall_dims.l(1)-obj.hall_dims.w(2)];
           region1.M = eye(2);
           region1.lims.left = -(obj.hall_dims.w(1)-obj.dims.wall_thickness);
           if obj.with_obs
%               region1.lims.upper = obj.hall_dims.l(1)-obj_width_points(1)*obj.hall_dims.w(2);
           end
           
           obj.regions{1} = region1;
           
           if obj.with_obs
               xlim = [obj.dims.wall_thickness ((obj.hall_dims.l(2)-obj.hall_dims.w(1))*obj_length_points(1))+obj.hall_dims.w(1)];
               ylim = [obj.hall_dims.l(1)-obj.hall_dims.w(2), obj.hall_dims.l(1)];
               region2.lims.x = xlim([1 1 2 2 1]);
               region2.lims.y = ylim([1 2 2 1 1]);
               region2.corner = [xlim(2) ylim(2)-obj_width_points(1)*obj.hall_dims.w(2)];
               region2.M = [0 1;1 0];
               obj.regions{2} = region2;
               
               xlim = [((obj.hall_dims.l(2)-obj.hall_dims.w(1))*obj_length_points(1)) ((obj.hall_dims.l(2)-obj.hall_dims.w(1))*obj_length_points(2))]+obj.hall_dims.w(1);
               ylim = [obj.hall_dims.l(1)-obj.hall_dims.w(2), obj.hall_dims.l(1)];
               region3.lims.x = xlim([1 1 2 2 1]);
               region3.lims.y = ylim([1 2 2 1 1]);
               region3.corner = [xlim(2) ylim(2)-obj_width_points(1)*obj.hall_dims.w(2)];
               region3.M = [0 1;1 0];
               obj.regions{3} = region3;
               
               xlim = [(obj.hall_dims.l(2)-obj.hall_dims.w(1))*obj_length_points(2) obj.hall_dims.l(2)];
               ylim = [obj.hall_dims.l(1)-obj.hall_dims.w(2), obj.hall_dims.l(1)];
               region4.lims.x = xlim([1 1 2 2 1]);
               region4.lims.y = ylim([1 2 2 1 1]);
               region4.corner = [xlim(2) ylim(2)];
               region4.M = [0 1;1 0];
               obj.regions{4} = region4;
           else
               xlim = [obj.dims.wall_thickness, obj.hall_dims.l(2)];
               ylim = [obj.hall_dims.l(1)-obj.hall_dims.w(2), obj.hall_dims.l(1)];
               region2.lims.x = xlim([1 1 2 2 1]);
               region2.lims.y = ylim([1 2 2 1 1]);
               region2.corner = [obj.hall_dims.l(2), obj.hall_dims.l(1)];
               region2.M = [0 1;1 0];
               region2.lims.left = -obj.hall_dims.w(2);
               obj.regions{2} = region2;
           end
           
           % Define total region
           xlims1 = [0 obj.hall_dims.w(1)] + obj.dims.wall_thickness;
           ylims1 = [0 obj.hall_dims.l(1)];
           first_hallway = polyshape(xlims1([1 2 2 1 1]),ylims1([1 1 2 2 1]));
           xlims2 = [0 100*obj.hall_dims.l(2)]+obj.dims.wall_thickness;
           ylims2 = obj.hall_dims.l(1) + [0 -obj.hall_dims.w(2)];
           second_hallway = polyshape(xlims2([1 2 2 1 1]),ylims2([1 1 2 2 1]));
           obj.total_region.polygon = union([first_hallway second_hallway]);
           verts = obj.total_region.polygon.Vertices;
           obj.total_region.xs = verts(:,1);
           obj.total_region.ys = verts(:,2);
       end
       function end_flag_check(obj,robot)
            % true when time to end
            in2ndregion = inpolygon(robot.position.x,robot.position.y,obj.regions{end}.lims.x,obj.regions{end}.lims.y);
            x_condition = robot.position.x > obj.hall_dims.l(2)-1;
            obj.end_flag = in2ndregion && x_condition;
       end
   end
end