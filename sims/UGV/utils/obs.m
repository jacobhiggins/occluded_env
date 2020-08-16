classdef obs < handle
   properties
      x
      y
      w
      h
      sec
      avoid % Should the obs avoid left or right?
      wypts
      xs_map
      ys_map
   end
   methods
%        function set_pos(obj,x,y)
%            obj.xpos = x;
%            obj.ypos = y;
%        end
       function set_pos(obj,xc,yc,M,hw,hl,fracw,fracl)
           % pass hallway length and width as a parameter
           % use frac to determine where along width/length you want obj
           try
               hw2 = hw - obj.w; % Use hw2 and hl2 to make sure obs doesn't collide with wall
               hl2 = hl - obj.h;
           catch e
               disp("You must define obs.w and obs.h first")
               return;
           end
           obj.x = -fracw*hw2 - obj.w/2; % x position in vertical hallway positioning
           obj.y = -fracl*hl2 - obj.h/2; % y pos in vertical hallway
           obj.set_avoid(hw);
           % Rotate obj
           [obj.x,obj.y] = u2c(obj.x,obj.y,xc,yc,M);
       end
       function [xpos,ypos] = get_pos(obj)
          xpos = obj.x;
          ypos = obj.y;
       end
       function set_sec(obj,sec)
           obj.sec = sec;
       end
       function sec = get_sec(obj)
          sec = obj.sec; 
       end
       function set_dims(obj,w,h)
          obj.w = w;
          obj.h = h;
       end
       function [w,h] = get_dims(obj)
          w = obj.w;
          h = obj.h;
       end
       function set_avoid(obj,hw)
          fracw = abs(obj.x)/hw;
          if fracw < 0.5
             obj.avoid = -1; % 0 = go left
          elseif fracw >= 0.5
             obj.avoid = 1;  % 1 = go right
          end
       end
       function set_wypts(obj,hw)
           wpt1 = [obj.avoid*(hw/2-obj.w/2),-obj.h];
           wpt2 = [obj.avoid*hw/2,-obj.h/2]; % If avoid left, wpts set to left of obj
           wpt3 = [obj.avoid*hw/2,obj.h/2]; % If avoid right, wpts set to right of obj
           wpt4 = [obj.avoid*(hw/2-obj.w/2),obj.h];
           obj.wypts = {wpt1,wpt2,wpt3,wpt4};
       end
       function set_map_points(obj,map)
           x1 = -obj.w/2; % x coordinate of lower left
           y1 = -obj.h/2; % y coordinate of lower left
           x2 = obj.w/2; % upper right
           y2 = obj.h/2; % upper right
           [x1,y1] = u2c(x1,y1,obj.x,obj.y,map.sections{obj.get_sec()});
           [x2,y2] = u2c(x2,y2,obj.x,obj.y,map.sections{obj.get_sec()});
           obj.xs_map = [x1,x2];
           obj.ys_map = [y1,y2];
       end
   end
end