classdef map_real < handle
   properties
       corner = struct("x",0.0,"y",0.0,"sub",[]);
       geoms = {};
       stop = false;
       model_traj = struct("wypt_bases",struct("xs",[],"ys",[]),"offset",1);
   end
   methods
       function init_params(obj,p)
           try
               obj.corner.sub = rossubscriber("/vicon/occ_corner/occ_corner");
               obj.get_corner_pos();
           catch
               disp("No corner object is published on vicon...");
           end
           obj.get_model_traj(p);
       end
       function get_corner_pos(obj)
           try
               corner_msg = receive(obj.corner.sub,1);
               obj.corner.x = corner_msg.Transform.Translation.X;
               obj.corner.y = corner_msg.Transform.Translation.Y;
           catch
               disp("Occluding corner data not received from vicon...");
               obj.corner.x = 20;
               obj.corner.y = 10;
           end
           obj.geoms{1}.corner = obj.corner;
           obj.geoms{1}.M_mpc = eye(2);
           obj.geoms{2}.corner = struct("x",10,"y",10);
           obj.geoms{2}.M_mpc = [0 1;1 0];
       end
       function get_model_traj(obj,p)
           % Initial wypt base
           obj.model_traj.wypt_bases.xs(1) = p.position.x;
           obj.model_traj.wypt_bases.ys(1) = p.position.y;
           % Second wypt base
           obj.model_traj.wypt_bases.xs(2) = p.position.x;
           obj.model_traj.wypt_bases.ys(2) = obj.corner.y + obj.model_traj.offset;
           % Third wypt base
           obj.model_traj.wypt_bases.xs(3) = p.position.x + 10;
           obj.model_traj.wypt_bases.ys(3) = obj.corner.y + obj.model_traj.offset;
       end
       function check_stop_experiment(obj,p)
           p.get_joy();
           if p.joy_buttons(p.JOY_O)
               obj.stop = true;
           end
       end
   end
end