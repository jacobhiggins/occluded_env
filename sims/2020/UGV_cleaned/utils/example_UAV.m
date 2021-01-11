classdef example_UAV < UAV
   methods
       function obj = example_UAV(map)
          obj = obj@UAV(map);
          obj.physical.mass = 1;
       end
   end
end