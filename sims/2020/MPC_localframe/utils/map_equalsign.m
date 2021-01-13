classdef map_equalsign < map
   properties
       wall_dims = struct("w",1.0,"h",1.0,"separation",1.0,"robot_offset",1.0);
   end
   methods
       function obj = map_equalsign()
           obj = obj@map();
           obj.dims.width = 10;
           obj.dims.height = 10;
           obj.dims.resolution = 10;
           obj.dims.dx = 1/obj.dims.resolution;
           obj.dims.wall_thickness = obj.dims.dx;
           
           % Robot suggest pose to start
           obj.robot_suggest.pose.x = obj.dims.width/2 - obj.wall_dims.w/2 - obj.wall_dims.robot_offset;
           obj.robot_sugesst.pose.y = obj.dims.height/2 - obj.wall_dims.separation/2 - obj.wall_dims.h - obj.wall_dims.robot_offset;
           
           % Bottom wall
           wall = struct("xs",[],"ys",[]);
           xs = obj.dims.width/2 + [-obj.wall_dims.w,obj.wall_dims.w]/2;
           ys = obj.dims.height/2 + [-obj.wall_dims.separation/2,-obj.wall_dims.separation/2-obj.wall_dims.h];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Top wall
           xs = obj.dims.width/2 + [-obj.wall_dims.w,obj.wall_dims.w]/2;
           ys = obj.dims.height/2 + [obj.wall_dims.separation/2,obj.wall_dims.separation/2obj.wall_dims.h];
           wall.xs = xs([1 2 2 1 1]);
           wall.ys = ys([1 1 2 2 1]);
           obj.walls{end+1} = wall;
           
           % Limits, for plotting
           obj.dims.xlim = [0 obj.dims.width];
           obj.dims.ylim = [0 obj.dims.height];
           
           % Reference trajectory definition
           obj.ref_traj.base_points.x = [obj.robot_suggest.pose.x,...
               obj.robot_suggest.pose.x,...
               obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
               obj.dims.width/2+obj.wall_dims.w/2+obj.wall_dims.robot_offset,...
               0];
           obj.ref_traj.base_points.y = [obj.robot_suggest.pose.y,...
               obj.dims.height/2-obj.wall_dims.separation/2-obj.wall_dims.h-obj.wall_dims.robot_offset,...
               obj.dims.height/2-obj.wall_dims.separation/2-obj.wall_dims.h-obj.wall_dims.robot_offset,...
               obj.dims.height/2,...
               obj.dims.height/2];
           % Complete set of points for ref traj, for waypoint shifting
           % based on safety
           
           
           % 
       end
   end
end