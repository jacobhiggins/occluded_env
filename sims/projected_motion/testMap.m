classdef testMap < handle
   properties
      wypts= {};
      plt_pm;
      plt_cmd;
      plt_wypt;
      plt_proj;
      plts;
      fig_main;
      pose_start = struct("x",0.0,"y",0.0);
      sim_end = false;
      vid;
   end
   methods
       function setParams_box(obj,h,w)
          if isempty(h)
              h = 3;
          end
          if isempty(w)
              w = 3;
          end
          wypt1 = struct("x",0,"y",h);
          wypt2 = struct("x",w,"y",h);
          wypt3 = struct("x",w,"y",0);
          wypt4 = struct("x",0,"y",0);
          obj.wypts = {wypt1,wypt2,wypt3,wypt4};
          obj.pose_start = struct("x",0.0,"y",0.0);
       end
       function init_plot(obj,p)
           obj.fig_main = gcf;
           hold on;
           obj.plt_pm = plot(p.x,p.y,'ro','MarkerFaceColor','blue','DisplayName','Point Mass');
           obj.plt_wypt = plot(p.wypt.x,p.wypt.y,'b*','DisplayName','waypoint');
           obj.plt_proj = plot([0 0],[0 0],'r-','LineWidth',3,'DisplayName','Projected Path');
           obj.plt_cmd = quiver(p.x,p.y,0,0,'LineWidth',2,'DisplayName',sprintf("Commanded %s",p.cmd_input.name));
           obj.plts = [obj.plt_pm,obj.plt_wypt,obj.plt_proj];
           axis equal;
           xlim([-1,12]);
           ylim([-1,12]);
           legend(obj.plts,'Location','eastoutside');
           title(obj.fig_main.CurrentAxes,sprintf("MPC Motion, Time: %0.2f",p.t));
           obj.vid = VideoWriter('vid.avi','Motion JPEG AVI');
           open(obj.vid);
       end
       function update_plot(obj,p)
           obj.plt_pm.XData = p.x;
           obj.plt_pm.YData = p.y;
           obj.plt_wypt.XData = p.wypt.x;
           obj.plt_wypt.YData = p.wypt.y;
           obj.plt_proj.XData = p.proj_mot.x;
           obj.plt_proj.YData = p.proj_mot.y;
           obj.plt_cmd.XData = p.x;
           obj.plt_cmd.YData = p.y;
           obj.plt_cmd.UData = p.cmd_input.x;
           obj.plt_cmd.VData = p.cmd_input.y;
           obj.end_flag(p);
           drawnow;
           frame = getframe(obj.fig_main);
           writeVideo(obj.vid,frame);
       end
       function end_flag(obj,p)
          if std(p.proj_mot.y) < 0.008 && p.y > 9
              obj.sim_end = true;
          end
       end
       function close_plot(obj)
          close(obj.vid); 
       end
   end
end