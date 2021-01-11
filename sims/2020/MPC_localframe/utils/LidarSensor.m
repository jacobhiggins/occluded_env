classdef LidarSensor < handle
   properties
      maxRange = 10;
      num_sensors = 200;
      angle_range = [-pi,pi]
      angles = [];
      ranges = [];
      endpoints = struct("x",[],"y",[],"relative_max",struct("x",[],"y",[]));
      freespace = struct("x",[],"y",[],"plot",struct("x",[-100],"y",[-100]));

   end
   methods
       function obj = LidarSensor()
          obj.angles = obj.angle_range(1):(obj.angle_range(2)-obj.angle_range(1))/(obj.num_sensors-1):obj.angle_range(2);
          obj.ranges = obj.maxRange*ones(1,obj.num_sensors);
          obj.endpoints.relative_max.x = obj.maxRange*cos(obj.angles);
          obj.endpoints.relative_max.y = obj.maxRange*sin(obj.angles);
       end
       function measure(obj,robot,map)
          for j = 1:obj.num_sensors
              [x_end,y_end] = u2c(obj.endpoints.relative_max.x(j),obj.endpoints.relative_max.y(j),0,0,robot.position.M);
              x = [0 x_end];
              y = [0 y_end] ;
              r_min = obj.maxRange;
              intersection_points = [];
              for i = 1:length(map.walls)
%                   if i~=3
%                      continue 
%                   end
                  wall = map.walls{i};
                  [xi,yi] = polyxpoly(x,y,wall.xs-robot.position.x,wall.ys-robot.position.y);
                  if ~isempty(xi)
                      intersection_points = [intersection_points;([xi,yi]+[robot.position.x,robot.position.y])];
                      rs = sqrt(xi.*xi + yi.*yi);
                      r_min_maybe = min(rs);
                      if r_min_maybe < r_min
                          r_min = r_min_maybe;
                      end
                  end
%                   rs = sqrt(xi.*xi + yi.*yi);
%                   r_min_maybe = min(rs);
%                   if ~isempty(r_min_maybe)
%                       if r_min_maybe < r_min
%                           r_min = r_min_maybe;
%                       end
%                   end
              end
              obj.ranges(j) = r_min;
          end
          obj.endpoints.x = obj.ranges.*cos(obj.angles+robot.position.theta) + robot.position.x;
          obj.endpoints.y = obj.ranges.*sin(obj.angles+robot.position.theta) + robot.position.y;
          robot.perception.visible_area.x = obj.endpoints.x;
          robot.perception.visible_area.y = obj.endpoints.y;
          robot.perception.FOV.x = obj.maxRange.*cos(obj.angles+robot.position.theta) + robot.position.x;
          robot.perception.FOV.y = obj.maxRange.*sin(obj.angles+robot.position.theta) + robot.position.y;
          obj.find_freespace(robot);
       end
       function find_freespace(obj,robot)
          % Calculate the lidar jumps that correspond to free space
          ranges = obj.ranges;
          ranges_shifted = [ranges(2:end) ranges(1)];
          endpoints = obj.endpoints;
          endpoints_shifted.x = [obj.endpoints.x(2:end) obj.endpoints.x(1)];
          endpoints_shifted.y = [obj.endpoints.y(2:end) obj.endpoints.y(1)];
          del_r = abs(ranges-ranges_shifted);
          idxs = or(del_r>0.5,and(del_r<0.001,ranges>=0.999*obj.maxRange));
          % Columns are (2) values for single line segments
          obj.freespace.x = [endpoints.x(idxs);...
              endpoints_shifted.x(idxs)];
          obj.freespace.y = [endpoints.y(idxs);...
              endpoints_shifted.y(idxs)];
          plot.x = [];
          plot.y = [];
          for i = 1:size(obj.freespace.x,2)
              plot.x = [plot.x obj.freespace.x(:,i)' NaN];
              plot.y = [plot.y obj.freespace.y(:,i)' NaN];
          end
          
          obj.freespace.plot = plot;
       end
   end
end