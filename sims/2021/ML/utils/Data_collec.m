classdef Data_collec < handle
   properties
      collect_on = true;
      collect_stats = true;
      times = [];
      positions = struct("x",[],"y",[]);
      vels = struct("x",[],"y",[],"norm",[],"zero_cross",0);
      cmds = [];
      KUs = struct("poly",[],...
          "area",[]);
      stats = struct("time2clear",0.0,...
          "max_KU",0.0,...
          "overshoot",0.0,...
          "vx_zero_cross",0,...
          "stuck",0);
   end
   methods
       function obj = Data_collec(robot)
           obj.times = robot.time;
           obj.positions.x = robot.position.x;
           obj.positions.y = robot.position.y;
           obj.vels.x = 0.0;
           obj.vels.y = 0.0;
           obj.vels.norm = 0.0;
           obj.vels.zero_cross = 0;
           obj.KUs.area = 0.0;
           obj.cmds = [0 0];
       end
       function record(obj,robot)
           persistent vx_sign_prev;
           if isempty(vx_sign_prev)
              vx_sign_prev = 0; 
           end
           obj.times = [obj.times robot.time];
           obj.positions.x = [obj.positions.x robot.position.x];
           obj.positions.y = [obj.positions.y robot.position.y];
           obj.vels.x = [obj.vels.x robot.vel.x];
           obj.vels.y = [obj.vels.y robot.vel.y];
           obj.vels.norm = [obj.vels.norm norm([robot.vel.x,robot.vel.y])];
           obj.cmds = [obj.cmds;robot.outer_cmd.vals];
           obj.KUs.area = [obj.KUs.area robot.perception.KU.area];
           if abs(robot.vel.x)>0.001
              vx_sign = sign(robot.vel.x);
              if abs(vx_sign_prev - vx_sign)>0.001
                 if abs(vx_sign_prev)>0.001
%                      fprintf("Time: %f, zero-crossing!\n",robot.time);
                     obj.vels.zero_cross = obj.vels.zero_cross + 1;
                 end
                    vx_sign_prev = vx_sign;
              end
           end
       end
       function record_stats(obj,robot,map)
           if robot.perception.KU.area > obj.stats.max_KU
               obj.stats.max_KU = robot.perception.KU.area;
           end
           % If the robot can't clear first section in 15 seconds, then its
           % stuck
           if robot.position.section==1 && robot.time>15.0
              obj.stats.stuck = 1;
              obj.stats.time2clear = robot.time;
              map.end_flag = true;
           end
           try
               if map.end_flag && (obj.stats.time2clear < 0.1)
                   obj.stats.time2clear = robot.time;
               end
           end
           obj.stats.vx_zero_cross = obj.vels.zero_cross;
       end
   end
end

% 12/2/2020
% Adding vx_zero_crossings as a variable data collector keeps track of
% This is used in cost funciton to penalizing changing direction too much