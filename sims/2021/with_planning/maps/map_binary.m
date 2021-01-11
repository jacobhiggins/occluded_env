classdef map_binary < map
    properties
        map_info
        algorithm
    end
    methods
        function obj = map_binary(str,path_planning)
            obj = obj@map();
            obj.dims.resolution = 2;
            load("binary_maps.mat");
            try
                % TODO: load binary_maps with information like resolution,
                % goal point, start pose, etc.
                obj.occ_map = binaryOccupancyMap(binary_maps(str),obj.dims.resolution);
            catch ME
                disp("The following exception was thrown:");
                disp(ME);
                disp("Please pass valid map name into constructor. Valid names include:");
                map_names = keys(binary_maps);
                for i = 1:length(keys(binary_maps))
                    map_names{i}
                end
            end
            % Suggested values for robot to start
            obj.robot_suggest.pose_start.x = 2;
            obj.robot_suggest.pose_start.y = 2;
            % Use path planning algorithm to generate reference trajectory
            obj.plan_path(path_planning);
            % Define regions for MPC
            xlims = [0 100];
            ylims = [0 100];
            region1.lims.x = xlims([1 1 2 2 1]);
            region1.lims.y = ylims([1 2 2 1 1]);
            region1.width = 100;
            region1.corner = [100,100];
            region1.M = eye(2);
            region1.lims.left = -100;
            region1.localframe.wypt_base.x = obj.ref_traj.base_points.x(1);
            region1.localframe.wypt_base.y = obj.ref_traj.base_points.y(1);
            region1.curl = 1;
            region1.top_constraint = 10;
            obj.regions{1} = region1;
        end
        function plan_path(obj,str)
            algorithms = ["prm","visibility"];
            if isequal(str,"visibility")
                obj.algorithm = visibility(obj.occ_map);
                obj.algorithm.get_corners();
%                 scatter(alorithm.corners(:,1),alorithm.corners(:,2))
                obj.algorithm.make_roadmap();
                start = [1,1];
                goal = [12,8];
                path = obj.algorithm.find_path(start,goal);
            elseif isequal(str,"prm")
                
            else
               disp("Please use valid path planning algorithm. These include:");
               for i = 1:length(algorithms)
                  algorithms(i) 
               end
               return;
            end
            obj.ref_traj.base_points.x = path(:,1);
            obj.ref_traj.base_points.y = path(:,2);
        end
        function end_flag_check(obj,robot)
            
        end
    end
    
end