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
           persistent last_sample_time;
           if isempty(last_sample_time)
              last_sample_time = toc; 
           end
           if toc - last_sample_time < min(1/p.MPC_Hz,0.1)
              return; 
           end
           p.get_joy();
           if p.joy_buttons(p.JOY_O)
               obj.stop = true;
           end
           last_sample_time = toc;
       end
       function ku = knownunknown(obj,p)
           ku.area = 0;
           ku.poly.x = [-1000,-999,-999,-1000];
           ku.poly.y = [-1000,-1000,-999,-999];
           xc = p.corner.mpc.x;
           yc = p.corner.mpc.y;
           [x,y] = c2u(p.position.x,p.position.y,xc,yc,p.M_mpc);
           hw2 = abs(p.next_owall);
           p.ku = ku;
           if norm([x,y],2)>p.maxRad || y > 0.0
               return
           end
           % Get upper wall interception point
           try
               [xus,yus] = linecirc(0,hw2,x,y,p.maxRad);
               xu = xus(1);
               yu = yus(1);
               if xus(2) > xu
                   xu = xus(2);
                   yu = yus(2);
               end
           catch
               
           end
           
           % Get lower wall interception point
           try
               [xls,yls] = linecirc(0,0,x,y,p.maxRad);
               xl = xls(1);
               yl = yls(1);
               if xls(2) > xl
                   xl = xls(2);
                   yl = yls(2);
               end
           catch
               return
           end
           
           % Get mid point
           phi = atan2(-x,-y);
           R = (hw2 - y)/cos(phi);
           r = min(R,p.maxRad);
           xm = x + r*sin(phi);
           ym = y + r*cos(phi);
           m = p.M_mpc\[xm;ym] + [xc;yc];
           try
               u = p.M_mpc\[xu;yu] + [xc;yc];
           end
           try
               l = p.M_mpc\[xl;yl] + [xc;yc];
           end
           phi_1 = atan2(yl-y,xl-x);
           if xm < xu && ~isempty(xu)
               % Extra points for good curvature
               phi_2 = atan2(yu-y,xu-x);
               %                 del_phi = phi_2 - phi_1;
               extra = p.M_mpc\([x;y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
               
               ku.poly.x = [xc;m(1);u(1);extra(1,:)';l(1)];
               ku.poly.y = [yc;m(2);u(2);extra(2,:)';l(2)];
               
           elseif ~isempty(xl)
               phi_2 = atan2(ym-y,xm-x);
               extra = p.M_mpc\([x;y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
               
               ku.poly.x = [xc;m(1);extra(1,:)';l(1)];
               ku.poly.y = [yc;m(2);extra(2,:)';l(2)];
           end
           ku.area = polyarea(ku.poly.x,ku.poly.y);
           p.ku = ku;
        end
   end
end