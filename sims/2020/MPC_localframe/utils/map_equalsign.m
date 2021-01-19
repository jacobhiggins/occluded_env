classdef map_equalsign < map
   properties
       wall_dims = struct("w",7.0,"h",2.5,"separation",2.5,"robot_offset",0.5);
       traffic_direction = NaN;
       obstacle = struct("enable",true,"height",1,"width",3.5);
   end
   methods
       function obj = map_equalsign()
           obj = obj@map();
           obj.dims.width = 11;
           obj.dims.height = 10;
           obj.dims.resolution = 10;
           obj.dims.dx = 1/obj.dims.resolution;
           obj.dims.wall_thickness = obj.dims.dx;
           
           % Robot suggest pose to start
           obj.robot_suggest.pose_start.x = obj.dims.width/2 - obj.wall_dims.w/2 - obj.wall_dims.robot_offset;
           obj.robot_suggest.pose_start.y = obj.dims.height/2 - obj.wall_dims.separation/2 - obj.wall_dims.h - obj.wall_dims.robot_offset;
           
           % Bottom wall
           wall = struct("xs",[],"ys",[]);
           xs = obj.dims.width/2 + [-obj.wall_dims.w,obj.wall_dims.w]/2;
           ys = obj.dims.height/2 + [-obj.wall_dims.separation/2-obj.wall_dims.h,-obj.wall_dims.separation/2];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Top wall
           xs = obj.dims.width/2 + [-obj.wall_dims.w,obj.wall_dims.w]/2;
           ys = obj.dims.height/2 + [obj.wall_dims.separation/2,obj.wall_dims.separation/2+obj.wall_dims.h];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           if obj.obstacle.enable
               % Include obstacle in middle section if enabeled
               xs = obj.dims.width/2 + [-obj.obstacle.width/2,obj.obstacle.width/2];
               ys = obj.walls{2}.ys(1) - [obj.obstacle.height,0];
               wall.xs = xs([1 2 2 1 1]);
               wall.ys = ys([1 1 2 2 1]);
               obj.walls{end+1} = wall;
           end
           
           % Limits, for plotting
           obj.dims.xlim = [0 obj.dims.width];
           obj.dims.ylim = [0 obj.dims.height];
           
           % Reference trajectory definition
           if obj.obstacle.enable
               obj.ref_traj.base_points.x = [obj.robot_suggest.pose_start.x,...
                   obj.robot_suggest.pose_start.x,...
                   obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
                   obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
                   obj.walls{3}.xs(3),...
                   obj.walls{3}.xs(1),...
                   obj.dims.width/2-obj.wall_dims.w/2];
               obj.ref_traj.base_points.y = [obj.robot_suggest.pose_start.y,...
                   obj.dims.height/2+obj.wall_dims.separation/2+obj.wall_dims.h+obj.wall_dims.robot_offset,...
                   obj.dims.height/2+obj.wall_dims.separation/2+obj.wall_dims.h+obj.wall_dims.robot_offset,...
                   obj.dims.height/2,...
                   (obj.walls{1}.ys(3)+obj.walls{3}.ys(1))/2,...
                   (obj.walls{1}.ys(3)+obj.walls{3}.ys(1))/2,...
                   obj.dims.height/2];
           else
               obj.ref_traj.base_points.x = [obj.robot_suggest.pose_start.x,...
                   obj.robot_suggest.pose_start.x,...
                   obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
                   obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
                   obj.dims.width/2-obj.wall_dims.w/2];
               obj.ref_traj.base_points.y = [obj.robot_suggest.pose_start.y,...
                   obj.dims.height/2+obj.wall_dims.separation/2+obj.wall_dims.h+obj.wall_dims.robot_offset,...
                   obj.dims.height/2+obj.wall_dims.separation/2+obj.wall_dims.h+obj.wall_dims.robot_offset,...
                   obj.dims.height/2,...
                   obj.dims.height/2];
           end
           % Complete set of points for ref traj, for waypoint shifting
           % based on safety
           obj.get_complete_ref();
           
           % Define regions for MPC
           % Bottom left region
           xlim = [0 obj.dims.width/2-obj.wall_dims.w/2];
           ylim = [0 obj.walls{1}.ys(3)];
           region1.lims.x = xlim([1 1 2 2 1]);
           region1.lims.y = ylim([1 2 2 1 1]);
           region1.width = abs(xlim(2)-xlim(1));
           region1.motional_corner = [region1.lims.x(3),...
               region1.lims.y(3)];
           region1.occluded_corner = region1.motional_corner;
           region1.M = eye(2);
           region1.curl = 1; % +1 for clockwise motion
           region1.lims.left = -(obj.robot_suggest.pose_start.x);
           region1.localframe.wypt_vec = [0 1];
           region1.localframe.wypt_base.x = obj.ref_traj.base_points.x(1);
           region1.localframe.wypt_base.y = obj.ref_traj.base_points.y(1);
           region1.top_constraint = 10;
           ROI_xs = [obj.walls{1}.xs(1) obj.dims.width];
           ROI_ys = [obj.walls{1}.ys(3) obj.walls{2}.ys(1)];
           region1.ROI = polyshape(ROI_xs([1 1 2 2 1]),ROI_ys([1 2 2 1 1]));
           obj.regions{end+1} = region1;
           % Top left region
           xlim = [0 obj.dims.width/2-obj.wall_dims.w/2];
           ylim = [obj.walls{1}.ys(3) obj.walls{2}.ys(3)];
           region2.lims.x = xlim([1 1 2 2 1]);
           region2.lims.y = ylim([1 2 2 1 1]);
           region2.width = obj.regions{1}.width;
           region2.motional_corner = [region2.lims.x(3),...
               obj.walls{2}.ys(3)];
           region2.occluded_corner = region2.motional_corner;
           region2.M = eye(2);
           region2.curl = 1;
           region2.lims.left = region1.lims.left;
           region2.localframe.wypt_vec = [0 1];
           region2.localframe.wypt_base.x = obj.ref_traj.base_points.x(1);
           region2.localframe.wypt_base.y = obj.ref_traj.base_points.y(1);
           region2.top_constraint = 10;
           ROI_xs = [obj.walls{2}.xs(1) obj.dims.width];
           ROI_ys = [obj.walls{2}.ys(3) obj.dims.height];
           region2.ROI = polyshape(ROI_xs([1 1 2 2 1]),ROI_ys([1 2 2 1 1]));
           obj.regions{end+1} = region2;
           % Top middle region
           xlim = [0 obj.walls{2}.xs(3)];
           ylim = [obj.walls{2}.ys(3) obj.dims.height];
           region3.lims.x = xlim([1 1 2 2 1]);
           region3.lims.y = ylim([1 2 2 1 1]);
           region3.width = abs(region3.lims.y(3)-region3.lims.y(1));
           region3.motional_corner = [region3.lims.x(3),region3.lims.y(1)];
           region3.occluded_corner = region3.motional_corner;
           region3.M = [0 -1;1 0];
           region3.curl = 1;
           region3.lims.left = -(obj.dims.height-obj.ref_traj.base_points.y(2));
           region3.localframe.wypt_vec = [1 0];
           region3.localframe.wypt_base.x = obj.ref_traj.base_points.x(2);
           region3.localframe.wypt_base.y = obj.ref_traj.base_points.y(2);
           region3.top_constraint = 10;
           ROI_xs = [obj.walls{2}.xs(2) obj.dims.width];
           ROI_ys = [0 obj.dims.height];
           region3.ROI = polyshape(ROI_xs([1 1 2 2 1]),ROI_ys([1 2 2 1 1]));
           obj.regions{end+1} = region3;
           % Top right region
           xlim = [obj.walls{2}.xs(3) obj.dims.width];
           ylim = [obj.walls{2}.ys(1) obj.dims.height];
           region4.lims.x = xlim([1 1 2 2 1]);
           region4.lims.y = ylim([1 2 2 1 1]);
           region4.width = abs(region4.lims.x(3)-region4.lims.x(1));
           region4.motional_corner = [region4.lims.x(1) region4.lims.y(1)];
           region4.occluded_corner = region4.motional_corner;
           region4.M = -eye(2);
           region4.curl = 1;
           region4.lims.left = -(obj.dims.width-obj.ref_traj.base_points.x(3));
           region4.localframe.wypt_vec = [0 -1];
           region4.localframe.wypt_base.x = obj.ref_traj.base_points.x(3);
           region4.localframe.wypt_base.y = obj.ref_traj.base_points.y(3);
           region4.top_constraint = 10;
           ROI_xs = [obj.walls{2}.xs(1) obj.walls{2}.xs(3)];
           ROI_ys = [obj.walls{1}.ys(3) obj.walls{2}.ys(1)];
           ROI_middle_poly = polyshape(ROI_xs([1 1 2 2 1]),ROI_ys([1 2 2 1 1]));
           if obj.obstacle.enable
               % If obstacle present in middle hallway
               wall3_poly = polyshape(obj.walls{3}.xs,obj.walls{3}.ys); % get obstacle polygon
               ROI_middle_poly = subtract(ROI_middle_poly,wall3_poly); % subtract obstacle polygon from middle polygon
           end
           region4.ROI = ROI_middle_poly;
           obj.regions{end+1} = region4;
           if obj.obstacle.enable
               % Add regions for navigation around obstacle if needed
               % Region to the right of obstacle
               xlim = [obj.walls{3}.xs(3) obj.dims.width];
               ylim = [obj.walls{3}.ys(1) obj.walls{2}.ys(1)];
               region5.lims.x = xlim([1 1 2 2 1]);
               region5.lims.y = ylim([1 2 2 1 1]);
               region5.width = abs(region5.lims.y(3)-region5.lims.y(1));
               region5.motional_corner = [obj.walls{3}.xs(3),obj.walls{3}.ys(1)];
               region5.occluded_corner = region5.motional_corner;
               region5.M = -1*eye(2);
               region5.curl = 1;
               region5.lims.left = -region5.width;
               region5.localframe.wypt_vec = [obj.ref_traj.base_points.x(5)-obj.ref_traj.base_points.x(4),...
                   obj.ref_traj.base_points.y(5)-obj.ref_traj.base_points.y(4)];
               region5.localframe.wypt_vec = region5.localframe.wypt_vec/norm(region5.localframe.wypt_vec);
               region5.localframe.wypt_base.x = obj.ref_traj.base_points.x(4);
               region5.localframe.wypt_base.y = obj.ref_traj.base_points.y(4);
               region5.top_constraint = 10;
               region5.ROI = ROI_middle_poly;
               obj.regions{end+1}=region5;
               % Region below the obstacle, above bottom wall
               xlim = [obj.walls{3}.xs(1) obj.dims.width];
               ylim = [obj.walls{1}.ys(3) obj.walls{3}.ys(1)];
               region6.lims.x = xlim([1 1 2 2 1]);
               region6.lims.y = ylim([1 2 2 1 1]);
               region6.width = abs(region6.lims.y(3)-region6.lims.y(1));
               region6.motional_corner = [obj.walls{3}.xs(1) obj.walls{3}.ys(1)];
               region6.occluded_corner = region6.motional_corner;
               region6.M = [0 1;-1 0];
               region6.curl = 1;
               region6.lims.left = -(obj.ref_traj.base_points.y(5)-obj.walls{1}.ys(3));
               region6.localframe.wypt_vec = [-1 0];
               region6.localframe.wypt_base.x = obj.ref_traj.base_points.x(5);
               region6.localframe.wypt_base.y = obj.ref_traj.base_points.y(5);
               region6.top_constraint = 10;
               region6.ROI = ROI_middle_poly;
               obj.regions{end+1}=region6;
           end
           % Last region in middle hallway (between two walls)
           xlim = [obj.walls{1}.xs(1) obj.walls{1}.xs(2)];
           ylim = [obj.walls{1}.ys(1) obj.walls{2}.ys(2)];
           regionlast.lims.x = xlim([1 1 2 2 1]);
           regionlast.lims.y = ylim([1 2 2 1 1]);
           regionlast.width = abs(regionlast.lims.y(3)-regionlast.lims.y(1));
           regionlast.motional_corner = [regionlast.lims.x(1) regionlast.lims.y(2)];
           regionlast.occluded_corner = regionlast.motional_corner;
           regionlast.M = [0 1;-1 0];
           regionlast.curl = 1;
           regionlast.lims.left = -regionlast.width;
           if obj.obstacle.enable
               vec = [obj.ref_traj.base_points.x(end)-obj.ref_traj.base_points.x(end-1),...
                   obj.ref_traj.base_points.y(end)-obj.ref_traj.base_points.y(end-1)];
               regionlast.localframe.wypt_vec = vec/norm(vec);
           else
               regionlast.localframe.wypt_vec = [-1 0];
           end
           regionlast.localframe.wypt_base.x = obj.ref_traj.base_points.x(end-1);
           regionlast.localframe.wypt_base.y = obj.ref_traj.base_points.y(end-1);
           regionlast.top_constraint = 10;
           regionlast.ROI = ROI_middle_poly;
           obj.regions{end+1} = regionlast;
           % Define total region
           xs = [0 obj.dims.width];
           ys = [0 obj.dims.height];
           freespace = polyshape(xs([1 2 2 1 1]),ys([1 1 2 2 1]));
           for i = 1:length(obj.walls)
               wall_poly = polyshape(obj.walls{i}.xs,obj.walls{i}.ys);
               freespace = subtract(freespace,wall_poly);
           end
           obj.total_region.polygon = freespace;
           verts = obj.total_region.polygon.Vertices;
           obj.total_region.xs = verts(:,1);
           obj.total_region.ys = verts(:,2);
       end
       function end_flag_check(obj,robot)
           if norm([robot.position.x-obj.ref_traj.base_points.x(end),...
                   robot.position.y-obj.ref_traj.base_points.y(end)])<0.35
               obj.end_flag = true;
           end
       end
   end
end