classdef visibility < handle
    %VISIBILITY Summary of this class goes here
    %   Algorithm for calculating visibility graph for binary occupancy map
    
    properties
        map;
        roadmap;
        corners;
        aug_points;
        visualize_process = true;
    end
    
    methods
        function obj = visibility(map)
            %VISIBILITY Construct an instance of this class
            %   Detailed explanation goes here
            obj.map = map;
        end
        
        function get_corners(obj)
            mat = occupancyMatrix(obj.map);
            [m,n] = size(mat);
            for i = 2:m-1
               for j = 2:n-1
                   corner_detect = false;
                   offset = [];
                   if getOccupancy(obj.map,[i j],'grid')
                       %*****Local Matrix Layout*****%
                       % 1 | 4 | 7 %
                       % 2 | 5 | 8 %
                       % 3 | 6 | 9 %
                       % Upper left
                       corner1 = [5 6 8 9]';
                       corner11 = [5 6 8]';
                       % Lower right
                       corner2 = [1 2 4 5]';
                       corner22 = [2 4 5]';
                       % Upper right
                       corner3 = [2 3 5 6]';
                       corner33 = [2 5 6]';
                       % Lower left
                       corner4 = [4 5 7 8]';
                       corner44 = [4 5 8]';
                       cap = [5 6]';
                       cup = [4 5]';
                       right_point = [2 5]';
                       left_point = [5 8]';
                       del_pos = 1.01/obj.map.Resolution/2*[-1,1;...
                           1,-1;...
                           1,1;...
                           -1,-1];
                       corner_shapes = [corner1 corner2 corner3 corner4];
                       hollow_corner_shapes = [corner11 corner22 corner33 corner44];
                       local_mat = mat(i-1:i+1,j-1:j+1);
                       is = find(local_mat);
                       for k = 1:size(corner_shapes,2)
                          corner_shape = corner_shapes(:,k);
                          hollow_corner_shape = hollow_corner_shapes(:,k);
                          if isequal(corner_shape,is) || isequal(hollow_corner_shape,is)
                              corner_detect = true;
                              offset = del_pos(k,:);
                          end
                       end
                       if isequal(cap,is)
                          offset = [del_pos(1,:);del_pos(3,:)];
                          corner_detect = true;
                       elseif isequal(cup,is)
                          offset = [del_pos(2,:);del_pos(4,:)];
                          corner_detect = true;
                       elseif isequal(right_point,is)
                          offset = [del_pos(2,:);del_pos(3,:)];
                          corner_detect = true;
                       elseif isequal(left_point,is)
                          offset = [del_pos(1,:);del_pos(4,:)];
                          corner_detect = true;
                       end
                   end
                   if corner_detect
                       obj.corners = [obj.corners; grid2world(obj.map,[i j])+offset];
                   end
               end
            end
        end
        
        function make_roadmap(obj)
            %METHOD1 Summary of this method goes here
            %   Makes a visibility graph roadmap in freespace
            n = size(obj.corners,1); % Get all corners
            obj.roadmap = Inf*ones(n); % Initialize roadmap matrix with Inf
            for i = 1:n
               corner1 = obj.corners(i,:); % First corner
               for j = 1:n
                   if i==j
                      obj.roadmap(i,j) = 0.0;
                      continue 
                   elseif i>j
                      continue 
                   end
                   corner2 = obj.corners(j,:); % Second corner
                   value = obj.roadmap_value(corner1,corner2);
                   obj.roadmap(i,j) = value;
                   obj.roadmap(j,i) = value;
               end
            end
        end
        
        function value = roadmap_value(obj,p1,p2)
            % Get angle between two corners
            angle = atan2(p2(2) - p1(2),p2(1) - p1(1));
            % Define pose at corner 1 (for provided rayIntersection function)
            % Offset pose by a little so radar doesn't "hit" at corner1
            pose = [p1 0] + 0.01*[cos(angle) sin(angle) 0];
            % Define range as length between corners
            % Make range smaller so that lidar doesn't "hit" at
            % corner2
            range = 0.99*norm(p1-p2);
            % Find intersection points using given function
            intersectionPts = rayIntersection(obj.map,pose,angle,range);
            % If there are no intersection points, then the two
            % points are mutually visible from one another
            if isnan(intersectionPts(1))
                value = norm(p1-p2);
            else
                value = Inf; 
            end
        end
        
        function vis_roadmap(obj,points,roadmap)
           figure(1); % Assume map is on figure 1
           hold on;
           n = size(roadmap,1);
           % For elements in roadmap matrix
           for i = 1:n
              for j = 1:n
                 if i>=j
                    continue 
                 end
                 % If the element isn't infinite, then plot line between
                 % two points
                 if ~isinf(roadmap(i,j))
                    plot([points(i,1),points(j,1)],[points(i,2),points(j,2)],"r:") 
                 end
              end
           end
        end
        
        function path = find_path(obj,start,goal)
            % Use Dijkstra to find shortest path on visibility graph
            
            % First, add start and goal as a point to roadmap
            n = size(obj.corners,1);
            aug_roadmap = [Inf*ones(2) Inf*ones(2,n);...
                Inf*ones(n,2) obj.roadmap];
            aug_points = [start;goal;obj.corners];
            % Then, construct augment roadmap for start and end
            % 1 = start
            % 2 = end
            for i = 1:2
                point1 = aug_points(i,:);
                for j = 1:(n+2)
                    if i>=j
                        continue
                    end
                    point2 = aug_points(j,:);
                    value = obj.roadmap_value(point1,point2);
                    aug_roadmap(i,j) = value;
                    aug_roadmap(j,i) = value;
                end
            end
            
            if obj.visualize_process
                obj.vis_roadmap(aug_points,aug_roadmap);
                scatter(start(1),start(2),"c");
                scatter(goal(1),goal(2),"c");
            end
            
            % Perform Dijsktra to find shortest path from start to goal
            info.path = cell(n+2,1); % Shortest path to get from start node to current node
            info.visited = zeros(1,(n+2)); % Visited nodes (0 = unvisited)
            info.distances = Inf*ones(1,(n+2)); % distances to all nodes from start
            info.distances(1) = 0; % Initialize dist from start to start as 0
            
            while sum(info.visited)<(n+2)
                % Find unvisisted nodes
                candidates = Inf*ones(1,(n+2));
                % If node is visited, keep candidate dist = Inf
                for i = 1:(n+2)
                   if info.visited(i)==0
                       candidates(i) = info.distances(i);
                   end
                end
                % Make minimum distances to unvisited node as current node
                [currentDist, currentPoint] = min(candidates);
                % From current point, update distances to all its neighbors
                % if the new distance is smaller
                for i= 1:(n+2)
                   newDist = currentDist + aug_roadmap(currentPoint,i);
                   if newDist < info.distances(i)
                      info.distances(i) = newDist;
                      info.path{i} = [info.path{currentPoint} i];
                   end
                end
                % Mark current node as visited
                info.visited(currentPoint) = 1;
            end
            obj.aug_points = aug_points;
            path = obj.aug_points(info.path{2},:);
            
        end
        function vis_path(obj)
            path = [start;obj.aug_points(info.path{2},:)];
            plot(path(:,1),path(:,2),"g",...
                "LineWidth",2);
        end
    end
end

