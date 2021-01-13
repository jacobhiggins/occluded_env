classdef map_intersection < map
   properties
       hall_dims = struct("w",[3,3],"l",[10,20]);
       direction = 1; % 1 = up, -1 = down
       traffic_direction = 0; % 1 = up, -1 = down, 0 = both, NaN = no traffic
       traffic_maxvel = 1.3; % average walking speed
       traffic_dims = struct("x",10,"y",120);
       traffic_starting_prob = 0.5;
   end
   methods
       function obj = map_intersection()
           obj = obj@map();
           obj.dims.width = obj.hall_dims.l(1) + obj.dims.wall_thickness;
           obj.dims.height = obj.hall_dims.l(2) + obj.dims.wall_thickness;
           obj.dims.resolution = 10;
           obj.dims.dx = 1/obj.dims.resolution;
           obj.dims.wall_thickness = obj.dims.dx;
           
           obj.robot_suggest.pose_start.x = 0;
           obj.robot_suggest.pose_start.y = obj.hall_dims.l(2)/2;
           
           % Bottom left wall
           wall = struct("xs",[],"ys",[]);
           xs = [0 obj.hall_dims.l(1)-obj.hall_dims.w(2)];
           ys = [0 obj.hall_dims.l(2)-obj.hall_dims.w(1)]/2;
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Top left wall
           xs = [0 obj.hall_dims.l(1)-obj.hall_dims.w(2)]; % Same as bottom left wall
           ys = [(obj.hall_dims.l(2)+obj.hall_dims.w(1))/2 obj.hall_dims.l(2)+1];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Right wall
           xs = [-obj.dims.wall_thickness obj.dims.wall_thickness] + obj.hall_dims.l(1);
           ys = [0 obj.hall_dims.l(2)+1];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Limits, for plotting
           obj.dims.xlim = [0 obj.hall_dims.l(1)+obj.dims.wall_thickness];
           obj.dims.ylim = [0 (obj.hall_dims.l(2)+obj.dims.dx)];
           
           % Reference trajectory definition
           obj.ref_traj.base_points.x = [obj.robot_suggest.pose_start.x,...
               obj.hall_dims.l(1)-obj.hall_dims.w(2)/2,...
               obj.hall_dims.l(1)-obj.hall_dims.w(2)/2];
           obj.ref_traj.base_points.y = [obj.robot_suggest.pose_start.y,...
               obj.robot_suggest.pose_start.y,...
               obj.direction*30];
           % Complete set of points for ref traj, for waypoint shifting
           % based on safety
           xs = [];
           ys = [];
           num_points = 200;
           for i = 1:length(obj.ref_traj.base_points.x)-1
               vec = [obj.ref_traj.base_points.x(i+1),obj.ref_traj.base_points.y(i+1)] - ...
                   [obj.ref_traj.base_points.x(i),obj.ref_traj.base_points.y(i)];
               vec = vec/num_points;
               xs = [xs ((1:num_points)*vec(1)+obj.ref_traj.base_points.x(i))];
               ys = [ys ((1:num_points)*vec(2)+obj.ref_traj.base_points.y(i))];
           end
           obj.ref_traj.complete_points.x = xs;
           obj.ref_traj.complete_points.y = ys;
           
           % Define regions for MPC
           % First hallway
           xlim = [0 obj.hall_dims.l(1)-obj.hall_dims.w(2)];
           ylim = [-obj.hall_dims.w(1) obj.hall_dims.w(1)]/2 + obj.hall_dims.l(2)/2;
           region1.lims.x = xlim([1 1 2 2 1]);
           region1.lims.y = ylim([1 2 2 1 1]);
           region1.width = obj.hall_dims.w(1);
           region1.motional_corner = [obj.hall_dims.l(1)-obj.hall_dims.w(2),...
                   obj.hall_dims.l(2)/2+obj.direction*obj.hall_dims.w(1)/2];
           if ~(obj.traffic_direction==0)
               % if traffic is going up or down
               region1.occluded_corner = [obj.hall_dims.l(1)-obj.hall_dims.w(2),...
                   obj.hall_dims.l(2)/2-obj.traffic_direction*obj.hall_dims.w(1)/2];
               region1.M = [0 obj.direction;1 0];
               region1.curl = -obj.direction;
           else
               % if traffic is going both ways
               region1.occluded_corner = [obj.hall_dims.l(1)-obj.hall_dims.w(2),...
                   obj.hall_dims.l(2)/2+obj.hall_dims.w(1)/2];
               region1.M = [0 1;1 0];
               region1.curl = -1;
           end
           region1.lims.left = -(obj.hall_dims.w(1));
           region1.localframe.wypt_vec = [1,0];
           region1.localframe.wypt_base.x = obj.ref_traj.base_points.x(1);
           region1.localframe.wypt_base.y = obj.ref_traj.base_points.y(1);
           region1.top_constraint = 10;
           obj.regions{end+1} = region1;
           % Second hallway
           xlim = [obj.hall_dims.l(1)-obj.hall_dims.w(2) obj.hall_dims.l(1)];
           ylim = [-100 100];
           region2.lims.x = xlim([1 1 2 2 1]);
           region2.lims.y = ylim([1 2 2 1 1]);
           region2.width = obj.hall_dims.w(2);
           region2.occluded_corner = [obj.hall_dims.l(1) obj.direction*100];
           region2.motional_corner = region2.occluded_corner;
           region2.M = [1 0;0 obj.direction];
           region2.lims.left = -obj.hall_dims.w(2);
           region2.localframe.wypt_vec = [0,obj.direction];
           region2.localframe.wypt_base.x = obj.ref_traj.base_points.x(2);
           region2.localframe.wypt_base.y = obj.ref_traj.base_points.y(2);
           region2.curl = obj.direction;
           region2.top_constraint = 10;
           obj.regions{end+1} = region2;
           % Define total free space (for KU calculation)
           xlims1 = [0 obj.hall_dims.l(1)-obj.dims.wall_thickness];
           ylims1 = [-obj.hall_dims.w(1) obj.hall_dims.w(1)]/2 + obj.hall_dims.l(2)/2;
           first_hallway = polyshape(xlims1([1 2 2 1 1]),ylims1([1 1 2 2 1]));
           xlims2 = [obj.hall_dims.l(1)-obj.hall_dims.w(2) obj.hall_dims.l(1)-obj.dims.wall_thickness];
           ylims2 = [-100 100] + obj.hall_dims.l(2);
           second_hallway = polyshape(xlims2([1 2 2 1 1]),ylims2([1 1 2 2 1]));
           obj.total_region.polygon = union([first_hallway second_hallway]);
           verts = obj.total_region.polygon.Vertices;
           obj.total_region.xs = verts(:,1);
           obj.total_region.ys = verts(:,2);
           
           % Define regions for traffic probabilities
           traffic_region = struct("lims",struct("x",[obj.hall_dims.l(1)-obj.hall_dims.w(2) obj.hall_dims.l(1)-obj.dims.wall_thickness],...
               "y",[0 obj.hall_dims.l(2)]));
           traffic_region.width.x = obj.traffic_dims.x;
           traffic_region.width.y = obj.traffic_dims.y;
           traffic_region.delx = (traffic_region.lims.x(2)-traffic_region.lims.x(1))/traffic_region.width.x;
           traffic_region.dely = (traffic_region.lims.y(2)-traffic_region.lims.y(1))/traffic_region.width.y;
           xs = (traffic_region.lims.x(1)+traffic_region.delx/2):traffic_region.delx:(traffic_region.lims.x(2)-traffic_region.delx/2);
           ys = (traffic_region.lims.y(1)+traffic_region.dely/2):traffic_region.dely:(traffic_region.lims.y(2)-traffic_region.dely/2);
           [X,Y] = meshgrid(xs,ys);
           traffic_region.X = X; % meshgrid of X values over traffic region
           traffic_region.Y = Y; % meshgrid of Y values over traffic region
           traffic_region.starting_prob = obj.traffic_starting_prob;
           traffic_region.occ.current = traffic_region.starting_prob*ones(size(traffic_region.X));
           traffic_region.occ.future = traffic_region.occ.current;
           % Define traffic flow for this region
           traffic_region.source_grid = zeros(size(X));
           motion_model_size = int16(obj.traffic_maxvel/(obj.update_Hz*traffic_region.dely));
           traffic_region.motion_model = zeros(2*motion_model_size+1,1);
           traffic_region.motion_indices = -motion_model_size:motion_model_size;
           if obj.traffic_direction == -1
               traffic_region.source_grid(end,:) = 1;
               traffic_region.motion_model(1:motion_model_size+1) = 1/double(motion_model_size+1);
           elseif obj.traffic_direction == 1
               traffic_region.source_grid(1,:) = 1;
               traffic_region.motion_model(motion_model_size+1:end) = 1/double(motion_model_size+1);
           elseif obj.traffic_direction == 0
               traffic_region.source_grid(1,:) = 1;
               traffic_region.source_grid(end,:) = 1;
               traffic_region.motion_model(:) = 1/double(2*motion_model_size + 1);
%                traffic_region.motion_model(motion_model_size+1) = 0.0;
           elseif isequal(obj.traffic_direction,NaN)
               
           end
           obj.traffic{1} = traffic_region;
       end
       function T = get_future_time(obj,robot)
           % returns how far into the future to convolve occupancy probs
           T = abs(obj.ref_traj.base_points.x(2) - robot.position.x)/robot.physical.vel_max;
%             T = min(max(obj.ref_traj.base_points.x(2)-robot.position.x,0),1);
%             T = (sqrt(robot.vel.x^2 + 2*abs(obj.ref_traj.base_points.x(2)-robot.position.x)) - robot.vel.x)/robot.physical.acc_max;
       end
       function end_flag_check(obj,robot)
            % true when time to end
            in2ndregion = inpolygon(robot.position.x,robot.position.y,obj.regions{end}.lims.x,obj.regions{end}.lims.y);
            if obj.direction==1
                y_condition = robot.position.y > obj.hall_dims.l(2);
            else obj.direction==(-1)
                y_condition = robot.position.y < 0;
            end
            if obj.end_flag==false
                obj.end_flag = in2ndregion && y_condition;
            end
       end
   end
end