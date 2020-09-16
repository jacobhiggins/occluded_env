classdef map_real < handle
   properties
       corner = struct("x",0.0,"y",0.0,"sub",[]);
   end
   methods
       function init_params(obj)
           obj.corner.sub = rossubscriber("/vicon/occ_corner/occ_corner");
           obj.get_corner_pos();
       end
       function get_corner_pos(obj)
           corner_msg = receive(obj.corner.sub,1);
           obj.corner.x = corner_msg.Transform.Translation.X;
           obj.corner.y = corner_msg.Transform.Translation.Y;
       end
   end
end