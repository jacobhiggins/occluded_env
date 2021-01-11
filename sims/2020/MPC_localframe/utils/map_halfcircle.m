classdef map_halfcircle < map
   properties
      
   end
   methods
       function obj = map_halfcircle()
           obj = obj@map();
           obj.dims.width = 12.5;
           obj.dims.height = 12.5;
           x0 = 2.5;
           y0 = 0;
           r = 10;
           s_range = 0:0.1:pi/2;
           obj.ref_traj.base_points.x = r*(1 - cos(s_range))+2.5;
           obj.ref_traj.base_points.y = r*(sin(s_range))+y0;
           obj.robot_suggest.pose_start.x = x0;
           obj.robot_suggest.pose_start.y = y0;
           
           % Limits, for plotting
           obj.dims.xlim = [0 obj.dims.width];
           obj.dims.ylim = [0 obj.dims.height];
           
           % Define regions for MPC
           
           xlim = [0 obj.dims.width];
           ylim = [0 obj.dims.height];
           region1.lims.x = xlim([1 1 2 2 1]);
           region1.lims.y = ylim([1 2 2 1 1]);
           region1.width = 2;
           region1.corner = [10,100];
           region1.M = eye(2);
           region1.lims.left = -5;
           region1.base.x = x0;
           region1.base.y = y0;
           region1.r = r; % postive 1 for clockwise motion
           region1.top_constraint = 4;
           
           obj.regions{1} = region1;
           
           
           % Define total region
           xlims1 = [0 10];
           ylims1 = [0 10];
           obj.total_region.xs = xlims1([1 2 2 1 1]);
           obj.total_region.ys = ylims1([1 1 2 2 1]);
       end
       function end_flag_check(obj,robot)
            % true when time to end
            in2ndregion = inpolygon(robot.position.x,robot.position.y,obj.regions{end}.lims.x,obj.regions{end}.lims.y);
            x_condition = robot.position.x > obj.dims.width-3;
            obj.end_flag = in2ndregion && x_condition;
       end
   end
end