classdef visualizer < handle
   properties
       fig
       format = struct("xlim",[-1.5 1.5],"ylim",[-3,3],"title","Visualizer");
       plt_rob
       plt_wall
       plt_wall_limit
       plt_trail
       plt_outline
       plt_radius
       plt_FOV
       plt_wypt
       plt_proj
       plt_ku
       plts = [];
       video = struct("record",true,"writer",VideoWriter("vid.avi"));
   end
   methods
       function init(obj,rob,map)
           close all;
           obj.fig = figure(1);
           hold on;
           
           obj.plt_wall = patch([map.corner.x,10,10,map.corner.x],[map.corner.y,map.corner.y,-10,-10],[0.5 0.5 0.5]);
           
           obj.plt_wall_limit = plot([rob.corner.mpc.x rob.corner.mpc.x],[rob.corner.mpc.y -10],'--');
           
           obj.plt_ku = fill(rob.ku.poly.x,rob.ku.poly.y,"r");
           
           obj.plt_rob = plot(rob.position.x,rob.position.y,"o",...
                              "MarkerSize",5,...
                              "DisplayName","Jackal3");
                          
           for i = 1:length(map.model_traj.wypt_bases.xs)-1
               plot([map.model_traj.wypt_bases.xs(i) map.model_traj.wypt_bases.xs(i+1)],...
                   [map.model_traj.wypt_bases.ys(i) map.model_traj.wypt_bases.ys(i+1)],"r--");
           end
           
           obj.plt_trail = plot(rob.trail.x, rob.trail.y,"DisplayName","Trail",...
                                "LineWidth",2);
                            
           obj.plt_outline = plot(rob.outline.x,rob.outline.y);
           
           circ_points = drawCirc(rob.r,rob.position.x,rob.position.y);
           obj.plt_radius = plot(circ_points.x,circ_points.y);
           circ_points = drawCirc(rob.maxRad,rob.position.x,rob.position.y);
           obj.plt_FOV = plot(circ_points.x,circ_points.y);
           
           obj.plt_wypt = plot(rob.wypt.x,rob.wypt.y,"x","MarkerSize",10,"DisplayName","Waypoint");
           
           obj.plt_proj = plot(rob.proj_mot.x,rob.proj_mot.y,...
               "DisplayName","Projected Motion",...
               "LineWidth",2);
           
           axis equal;
           xlim(obj.format.xlim);
           ylim(obj.format.ylim);
           
           if obj.video.record
              open(obj.video.writer);
              frame = getframe(gcf);
              writeVideo(obj.video.writer,frame);
           end
       end
       function update(obj,rob,map)
           map.knownunknown(rob);
           obj.plt_rob.XData = rob.position.x;
           obj.plt_rob.YData = rob.position.y;
           obj.plt_ku.XData = rob.ku.poly.x;
           obj.plt_ku.YData = rob.ku.poly.y;
           obj.plt_trail.XData = rob.trail.x;
           obj.plt_trail.YData = rob.trail.y;
           obj.plt_outline.XData = rob.outline.x;
           obj.plt_outline.YData = rob.outline.y;
           circ_points = drawCirc(rob.r,rob.position.x,rob.position.y);
           obj.plt_radius.XData = circ_points.x;
           obj.plt_radius.YData = circ_points.y;
           circ_points = drawCirc(rob.maxRad,rob.position.x,rob.position.y);
           obj.plt_FOV.XData = circ_points.x;
           obj.plt_FOV.YData = circ_points.y;
           obj.plt_wypt.XData = rob.wypt.x;
           obj.plt_wypt.YData = rob.wypt.y;
           obj.plt_proj.XData = rob.proj_mot.x;
           obj.plt_proj.YData = rob.proj_mot.y;
           if obj.video.record
              frame = getframe(gcf);
              writeVideo(obj.video.writer,frame);
           end
       end
       function close(obj)
           close(obj.video.writer);
           close all;
       end
   end
end