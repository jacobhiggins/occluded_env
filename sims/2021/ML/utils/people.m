classdef people < handle
    properties
        num = 1000;
%         num = 0;
        initial_patch = struct("x",struct("min",0.0,"max",0.0),"y",struct("min",0.0,"max",0.0));
        vel_profile = struct("x",struct("min",0.0,"max",0.0));
        positions = struct("x",[],"y",[]);
        vels = struct("x",[]);
        data = struct("rs",[]);
    end
    methods
        function init_params(obj,map)
            obj.initial_patch.x.min = 0;
            obj.initial_patch.x.max = map.hls(2)+1.2*map.maxRad_suggest;
            obj.initial_patch.y.min = map.hls(1)-map.hws(2);
            obj.initial_patch.y.max = map.hls(1);
            obj.vel_profile.x.min = -5.0;
            obj.vel_profile.x.max = 0.0;
            obj.fill_patch();
            obj.set_vels();
        end
        function fill_patch(obj)
            obj.positions.x = rand(obj.num,1)*(obj.initial_patch.x.max - obj.initial_patch.x.min) + obj.initial_patch.x.min;
            obj.positions.y = rand(obj.num,1)*(obj.initial_patch.y.max - obj.initial_patch.y.min) + obj.initial_patch.y.min;
        end
        function set_vels(obj)
            obj.vels.x = rand(obj.num,1)*(obj.vel_profile.x.max - obj.vel_profile.x.min) + obj.vel_profile.x.min;
        end
        function move(obj,dt)
            obj.positions.x = obj.positions.x + obj.vels.x*dt;
            revert_is = (obj.positions.x < 0);
            obj.positions.x(revert_is) = obj.initial_patch.x.max + (obj.positions.x(revert_is) - 0);
        end
        function sense(obj,p)
            ds = vecnorm([p.x-obj.positions.x,p.y-obj.positions.y],2,2);
            theta_vis = atan2(p.yc_mpc_r - p.y,p.xc_mpc_r - p.x);
            theta_ppl = atan2(obj.positions.y-p.y,obj.positions.x-p.x);
            is = logical( (ds < p.maxRad) .* ((theta_ppl>theta_vis)+(p.current_sec==2)));
            rs_sensed = ds(is);
            obj.data.rs = [obj.data.rs;rs_sensed];
%             if p.y < p.yc_mpc_r
%                 % If not passed corner, keep spawning people
%                 obj.positions.x(is) = obj.initial_patch.x.max;
%             else
%                 % else, kill them all
%                 obj.positions.x = [];
%                 obj.positions.y = [];
%                 obj.vels.x = [];
%             end
            obj.positions.x(is) = obj.initial_patch.x.max; % KEEP PEOPLE ALIVE AFTER PASSING CORNER
%             obj.positions.x = obj.positions.x(~is);
%             obj.positions.y = obj.positions.y(~is);
%             obj.vels.x = obj.vels.x(~is);
        end
    end
end