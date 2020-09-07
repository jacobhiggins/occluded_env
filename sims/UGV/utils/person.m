classdef person < handle
   properties
      state = struct("x",0.0,"y",0.0,"vx",0.0,"vy",0.0);
      size = 0.5; % size of human, meters
      plt; % plot handle
      collide = false;
      collisions = 0;
   end
   methods
       function setParams(obj,m)
           obj.state.y = m.hls(1) - m.hws(2) + 1.1*obj.size;
           obj.state.x = m.hws(1) + normrnd(7.3,0.5);
           obj.state.vy = 0;
           obj.state.vx = min(normrnd(-1,0.2),-0.5);
           obj.collide = false;
       end
       function init_plot(obj)
           hold on;
           obj.plt = plot(obj.state.x,obj.state.y,"bo","MarkerSize",10,"MarkerFaceColor","b");
       end
       function update_plot(obj)
          obj.plt.XData = obj.state.x;
          obj.plt.YData = obj.state.y;
       end
       function move(obj,m,dt)
          if obj.state.x >= 0 && ~obj.collide
              obj.state.x = obj.state.x + obj.state.vx*dt;
              obj.state.y = obj.state.y + obj.state.vy*dt;
          else
              if obj.collide
                 disp("Collision!"); 
              end
             obj.setParams(m); 
          end
       end
       function check_collision(obj,r)
           d = norm([obj.state.x-r.x,obj.state.y-r.y],2);
           if d < (obj.size + r.size)
               obj.collide = true;
               obj.collisions = obj.collisions + 1;
           end
       end
   end
end