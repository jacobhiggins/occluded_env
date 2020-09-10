% Three corridor map
% Reflects real dimensions of the jackal/lab arena layout
classdef mapA3 < map
    properties
       hls = [10,10]; % hw1 used to be 5.4-0.235, hw2 used to be 4-0.235
       hws = [2.5,2.5];
    end
    methods
        function setParams(obj)
           % End flag is false
           obj.end_flag = true;
           % Number of sections
           obj.num_secs = 3;
           obj.sections = cell(1,obj.num_secs);
           % Starting pose
           obj.pose_start.x = obj.hws(1)*0.5;
           obj.pose_start.y = 0.0;
           % Set section orientations
           obj.sections{1} = eye(2);
           obj.sections{2} = [0,1;1,0];
           obj.sections{3} = eye(2);
           % Set x,y locations of wall
           
           wall1.x = [obj.hws(1),obj.hls(2),obj.hls(2),obj.hws(1)];
           wall1.y = [0,0,obj.hls(1),obj.hls(1)];
           obj.walls{1} = wall1;
           box1.x = [obj.hws(1),obj.hls(2),obj.hls(2),obj.hws(1),obj.hws(1)];
           box1.y = [0,0,obj.hls(1)-obj.hws(2),obj.hls(1)-obj.hws(2),0];
           obj.boxs{1} = box1;
           
           wall2.x = [-obj.hws(1),obj.hls(2),obj.hls(2),-obj.hws(1)];
           wall2.y = [obj.hls(1),obj.hls(1),1.1*obj.hls(1),1.1*obj.hls(1)];
           obj.walls{2} = wall2;
           box2.x = [-obj.hws(1),obj.hls(2),obj.hls(2),-obj.hws(1)];
           box2.y = [obj.hls(1),obj.hls(1),1.1*obj.hls(1),1.1*obj.hls(1)];
           obj.boxs{2} = box2;
           
           wall3.x = [-obj.hws(1),0,0,-obj.hws(1)];
           wall3.y = [0,0,obj.hls(1),obj.hls(1)];
           obj.walls{3} = wall3;
           box3.x = [-obj.hws(1),0,0,-obj.hws(1),-obj.hws(1)];
           box3.y = [wall1.y wall1.y(end)];
           obj.boxs{3} = box3;
           % Set obstacles in map
           obj.set_obs();
           % Set waypoint bases
           obj.set_wypt_base();
           % Set xlim and ylim
           obj.plt_xlim = [-0.1*obj.hws(1),obj.hls(2)];
           obj.plt_ylim = [0,1.1*obj.hls(1)];
           % Set corner
           obj.set_corners();
           % Set patches of probabilities
           obj.set_patches();
           % Suggested maximum waypoint radius
           obj.maxRad_suggest = 5;
        end
        function set_wypt_base(obj)
            % Waypoint bases, n x 2
            % n = number of waypoint bases
            % (x,y)
            obj.wypt_bases = [obj.pose_start.x,obj.pose_start.y];
            obj.wypt_bases = [obj.wypt_bases;obj.pose_start.x,obj.hls(1)-obj.hws(2)/2];
            M2 = [0,1;1,0];
            for i = 1:length(obj.obss)
                obs = obj.obss{i};
                avoid1 = obs.avoid;
                x1 = obs.avoid*obs.w/2; % x coordinate of lower left
                y1 = -obs.h/2; % y coordinate of lower left
                x2 = x1; % x coordinate of upper left
                y2 = obs.h/2; % y coordinate of upper left
                [x1,y1] = u2c(x1,y1,obs.x,obs.y,M2);
                [x2,y2] = u2c(x2,y2,obs.x,obs.y,M2);
                y1 = (y1+(obj.hls(1) + obj.hws(2)*(avoid1-1)/2))/2; % Between object and wall of 2nd hallway
                y2 = (y2+(obj.hls(1) + obj.hws(2)*(avoid1-1)/2))/2;
                obj.wypt_bases = [obj.wypt_bases;x1,y1;x2,y2];
            end
            obj.wypt_bases = [obj.wypt_bases;obj.hls(2)+100,max(obj.hls(1)-obj.hws(2)/2,obj.wypt_bases(end,2))];
            obj.wypt_bases = [obj.wypt_bases;obj.hls(2)+100,obj.hls(1)-obj.hws(2)/2];
        end
        function wypt_bases = get_wypt_bases(obj)
            wypt_bases = obj.wypt_bases;
        end
        function set_obs(obj)
            obs_secs = [];
%             obs_secs = [2,2]; % Define section for obstacles
%             obs_secs = [2];
            obs_ls = [30,30]; % Define lengths of obstacles
            obs_ws = [15,15]; % Define widths of obstacles
            %     fracws = [1];
            %     fracls = [0.5];
            fracws = [1,1];
            fracls = [0.5,0.1];
            num_obs = length(obs_secs);
            obj.obss = cell(1,num_obs);
            for i = 1:num_obs
                sec = obs_secs(i);
                M = obj.sections{sec};
                obs_l = obs_ls(i);
                obs_w = obs_ws(i);
                fracw = fracws(i);
                fracl = fracls(i);
                [obs_xc,obs_yc] = setCorner(0,0,0,0,sec,obj.hws,obj.hls,{},{});
                o = obs(); % NOTE: Can't name instance of object the name of object type
                o.set_sec(sec);
                o.set_dims(obs_w,obs_l);
                o.set_pos(obs_xc,obs_yc,M,obj.hws(sec),obj.hls(sec),fracw,fracl);
                o.set_wypts(obj.hws(sec));
                o.set_map_points(obj);
                %        o.set_avoid(hws(sec));
                box.x = [o.xs_map(1),o.xs_map(2),o.xs_map(2),o.xs_map(1),o.xs_map(1)];
                box.y = [o.ys_map(1),o.ys_map(1),o.ys_map(2),o.ys_map(2),o.ys_map(1)];
                obj.boxs = cat(2,obj.boxs,box);
                obj.obss{i} = o;
            end
        end
        function set_corners(obj)
            obj.corners_r = [obj.hws(1),obj.hls(1)-obj.hws(2),1]; % xc, yc, avoid (+1 clockwise, -1 counterclockwise)
            obj.corners_l = [0,obj.hls(1)-obj.hws(2)]; % xc, yc, rc # (this is associated with first corner
%             M2 = [0,1;-1,0];
%             M3 = -1*eye(2);
            M4 = [0,1;1,0];
            obj.Ms_mpc = {eye(2)};
            obj.walls_mpc = [-1*obj.hws(1)+1];
            for i = 1:length(obj.obss)
                obs = obj.obss{i};
                avoid1 = obs.avoid; % +1 = move left to avoid, -1 = move right to avoid
                x1 = obs.avoid*obs.w/2; % x coordinate of lower left
                y1 = -obs.h/2; % y coordinate of lower left
                x2 = x1; % x coordinate of upper left
                y2 = obs.h/2; % y coordinate of upper left
                [x1,y1] = u2c(x1,y1,obs.x,obs.y,M4);
                [x2,y2] = u2c(x2,y2,obs.x,obs.y,M4);
                if avoid1 < 0
                    obj.corners_l = [obj.corners_l;x1,y1;x2,y2];
                else
                    obj.corners_r = [obj.corners_r;x1,y1,avoid1;x2,y2,avoid1];
                end
                obj.Ms_mpc = cat(2,obj.Ms_mpc,{[1,0;0,avoid1]});
                obj.Ms_mpc = cat(2,obj.Ms_mpc,{[0,-1*avoid1;1,0]});
                obj.walls_mpc = [obj.walls_mpc -30 -1*(obj.hws(2)-obs.w)];
            end
            % Corner at end of 2nd hallway
            obj.walls_mpc = [obj.walls_mpc -obj.hws(2)];
            obj.corners_r = [obj.corners_r;obj.hls(2)+100,obj.hls(1)-obj.hws(2),1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,[0,-1;1,0]);
            
            obj.corners_l = [obj.corners_l;obj.hls(2),obj.hls(1)];
%             
%             % Corner at end of last hallway
%             obj.walls_mpc = [obj.walls_mpc -obj.hws(3)];
%             obj.corners_r = [obj.corners_r;obj.hls(2)-obj.hws(3),-10000,1];
%             obj.corners_l = [obj.corners_l;obj.hls(2),0];
%             obj.Ms_mpc = cat(2,obj.Ms_mpc,eye(2));
        end
        function set_patches(obj)
            obj.patches.num = 50;
            obj.patches.xstart = -obj.hws(1);
            obj.patches.xend = obj.hls(2);
            obj.patches.ybottom = obj.hls(1)-obj.hws(2);
            obj.patches.ytop = obj.hls(1);
            obj.patches.probs = 0.5*ones(1,obj.patches.num);
            obj.patches.width = (obj.patches.xend - obj.patches.xstart)/obj.patches.num;
            obj.patches.centers = [(obj.patches.width)*(0:obj.patches.num-1)+obj.patches.width/2+obj.patches.xstart;...
                ((obj.patches.ytop+obj.patches.ybottom)/2)*ones(1,obj.patches.num)];
        end
        function check_flag(obj,p)
            if p.y > obj.hls(1)/2 && p.x > obj.hls(2)
               obj.end_flag = false; 
            end
        end
    end
end