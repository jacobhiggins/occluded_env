% Equal Sign
classdef mapB < map
    properties
       hls = struct("side",25,"top",100);
       hws = struct("top",10,"mid",10);
       room_w = 15;
    end
    methods
        function setParams(obj)
            obj.end_flag = true;
            obj.num_secs = 6;
            obj.sections = cell(1,obj.num_secs);
            obj.pose_start.x = obj.hls.side/2;
            obj.pose_start.y = obj.hws.top;
            obj.sections{1} = eye(2);
            obj.sections{2} = eye(2);
%             obj.sections{3} = eye(2);
            obj.sections{3} = [0 -1;1 0];
            obj.sections{4} = diag([-1,-1]);
            obj.sections{5} = diag([-1,-1]);
            obj.sections{6} = [0 1;-1 0];
            wall1.x = [obj.hls.side,obj.hls.top-obj.hls.side,obj.hls.top-obj.hls.side,obj.hls.side];
            wall1.y = [obj.hws.top,obj.hws.top,obj.hws.top+obj.room_w,obj.hws.top+obj.room_w];
            obj.walls{1} = wall1;
            wall2.x = [obj.hls.side,obj.hls.top-obj.hls.side,obj.hls.top-obj.hls.side,obj.hls.side];
            wall2.y = [obj.hws.top,obj.hws.top,obj.hws.top+obj.room_w,obj.hws.top+obj.room_w] + (obj.hws.mid+obj.room_w)*ones(1,4);
            obj.walls{2} = wall2;
            obj.set_wypt_base();
            obj.set_corners();
            obj.plt_xlim = [0,obj.hls.top];
            obj.plt_ylim = [0,2*obj.hws.top+obj.hws.mid+2*obj.room_w];
            obj.maxRad_suggest = 20;
        end
        % Since wypt bases is chosen as which wypt corner you're currently
        % at, you must include a waypoint base for each corner
        function set_wypt_base(obj)
            obj.wypt_bases = [obj.pose_start.x,obj.pose_start.y];
            obj.wypt_bases = [obj.wypt_bases;obj.hls.side/2,obj.hws.top+obj.room_w]; % NEW WAYPOINT BASE
            obj.wypt_bases = [obj.wypt_bases;obj.hls.side/2,1.5*obj.hws.top+2*obj.room_w+obj.hws.mid];
            obj.wypt_bases = [obj.wypt_bases;obj.hls.top-obj.hls.side/2,1.5*obj.hws.top+2*obj.room_w+obj.hws.mid];
            obj.wypt_bases = [obj.wypt_bases;obj.hls.top-obj.hls.side/2,obj.hws.top+obj.room_w+obj.hws.mid]; % NEW WAYPOINT BASEs
            obj.wypt_bases = [obj.wypt_bases;obj.hls.top-obj.hls.side/2,obj.hws.top/2];
            obj.wypt_bases = [obj.wypt_bases;obj.hls.side/2,obj.hws.top/2];
            obj.wypt_bases = [obj.wypt_bases;obj.hls.side/2,1.5*obj.hws.top+2*obj.room_w+obj.hws.mid];
        end
        function set_corners(obj)
            % First corner
            obj.corners = [obj.hls.side,obj.hws.top+obj.room_w,1];
            obj.Ms_mpc = {obj.sections{1}};
            obj.walls_mpc = -1*obj.hls.side;
            % Second corner
            obj.corners = [obj.corners;obj.hls.side,obj.hws.top+2*obj.room_w+obj.hws.mid,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{2});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hls.side];
            % Third corner
            obj.corners = [obj.corners;obj.hls.top-obj.hls.side,obj.hws.top+2*obj.room_w+obj.hws.mid,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{3});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hws.top];
            % Fourth corner
            obj.corners = [obj.corners;obj.hls.top-obj.hls.side,obj.hws.top+obj.room_w+obj.hws.mid,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{4});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hls.side];
            % Fifth corner
            obj.corners = [obj.corners;obj.hls.top-obj.hls.side,obj.hws.top,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{5});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hls.side];
            % Sixth corner
            obj.corners = [obj.corners;obj.hls.side,obj.hws.top,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{6});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hws.top];
            % Seventh (redundant) corner, same as first corner
            obj.corners = [obj.corners;obj.hls.side,obj.hws.top+obj.room_w,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{1});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hls.side];
            % Eigth (redundant) corner, same as second corner
            obj.corners = [obj.corners;obj.hls.side,obj.hws.top+2*obj.room_w+obj.hws.mid,1];
            obj.Ms_mpc = cat(2,obj.Ms_mpc,obj.sections{2});
            obj.walls_mpc = [obj.walls_mpc -1*obj.hls.side];
        end
        function check_flag(obj,p)
            if p.y < obj.hws.top && p.x < obj.hls.side
               obj.end_flag = false; 
            end
        end
    end
end