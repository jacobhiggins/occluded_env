classdef Data_collec < handle
   properties
      collect_on = true;
      collect_stats = true;
      times = [];
      positions = struct("x",[],"y",[]);
      vels = struct("x",[],"y",[],"norm",[]);
      cmds = [];
      KUs = struct("poly",[],...
          "area",[]);
      stats = struct("time2clear",0.0,...
          "max_KU",0.0,...
          "overshoot",0.0);
   end
   methods
       function obj = Data_collec(robot)
           obj.times = robot.time;
           obj.positions.x = robot.position.x;
           obj.positions.y = robot.position.y;
           obj.vels.x = 0.0;
           obj.vels.y = 0.0;
           obj.vels.norm = 0.0;
           obj.KUs.area = 0.0;
           obj.cmds = [0 0];
       end
       function record(obj,robot)
           obj.times = [obj.times robot.time];
           obj.positions.x = [obj.positions.x robot.position.x];
           obj.positions.y = [obj.positions.y robot.position.y];
           obj.vels.x = [obj.vels.x robot.vel.x];
           obj.vels.y = [obj.vels.y robot.vel.y];
           obj.vels.norm = [obj.vels.norm norm([robot.vel.x,robot.vel.y])];
           obj.cmds = [obj.cmds;robot.outer_cmd.vals];
           obj.KUs.area = [obj.KUs.area robot.perception.KU.area];
       end
       function record_stats(obj,robot,map)
           if robot.perception.KU.area > obj.stats.max_KU
               obj.stats.max_KU = robot.perception.KU.area;
           end
           if (robot.position.x > (map.hall_dims.w(1)+map.dims.wall_thickness)) && (obj.stats.time2clear < 0.1)
               obj.stats.time2clear = robot.time;
           end
       end
   end
end