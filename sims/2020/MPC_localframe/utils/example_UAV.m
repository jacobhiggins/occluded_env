classdef example_UAV < UAV
   methods
       function obj = example_UAV(map)
          obj = obj@UAV(map);
          obj.physical.mass = 1;
          obj.physical.acc_max = 2;
          obj.physical.vel_max = 2;
          obj.safety.stopping_dist = min(obj.physical.vel_max^2/(2*obj.physical.acc_max),0.9*obj.lidar.maxRange);
       end
   end
end