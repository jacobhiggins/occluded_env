% Defines interface superclass for maps
classdef map < handle
    properties
        maxRad_suggest % suggested maximum radius
        pose_start % Starting pose of robot
        walls % Walls of the hallway, for plotting mostly
        walls_mpc % Walls for the MPC
        num_secs % Number of sections
        sections % section orientations
        Ms_mpc % matrix for MPC calculation, for hallways + obstacles
        wypt_bases % waypoint bases for motion
        corners_r % Defined right corners of the map, orientation of how to avoid them
        corners_l % Defined left corners of the map
        obss % obstacles of the map
        patches % Patches of uncertainty
        end_flag = true; % true when the sim should end
        % **** Main Plots ****
        plt_p % graphic handle for point
        plt_wypt % graphic handle for waypoint
        plt_traj % graphic handle for model trajectory
        plt_trail % graphic handle for trail
        plt_proj % graphic handle for projected motion
        plt_cmd % graphic handle for commanded inputs
        plt_corner_mpc_r; % graphic handle for right corners
        plt_corner_mpc_l; % graphic handle for left corners
        plt_corner_wp;
        plts; % Running list of all plots
        plt_circ % graphic handle for waypoint circle
        plt_xlim % x limit for sim plot
        plt_ylim % y limit for sim plot
        show_trail = true; % showing trail
        show_circ = true; % boolean for showing/not showing circ
        show_proj = true; % s howing/not showing projected motion
        show_cmd = true; % showing/not showing commanded inputs
        show_cornerMPC = true; % show corner MPC
        show_cornerWP = true; % show corner waypoint
        show_legend = true; % show legend
        % **** Auxilary Plots ****
        plt_vxs % graphics handle for x velocities
        plt_vys % graphics handle for y velocities
        plt_uxs
        plt_uys
        plt_LOS
        % **** Recording Video ****
        fig_main
        vid
        rec_vid = true;
    end
    methods %(Abstract)
        setParams(obj); % Set properties of the map
        get_wypt_base(obj); % Get all waypoint bases for a map
        set_corners_r(obj);
%         termcond(obj,p) % returns true/false for map terminal condition
        function circ = get_circ(obj,p)
            theta = 0:0.1:2*pi;
            circ.x = p.r*cos(theta) + p.x;
            circ.y = p.r*sin(theta) + p.y;
        end
        function initial_plot(obj,p)
            hold on;
            % Save figure handle (for video)
            obj.fig_main = gcf;
            % Walls
            gray = [0.5 0.5 0.5];
            for i = 1:length(obj.walls)
               wall = obj.walls{i};
               patch(wall.x,wall.y,gray);
            end
            % Robot (point mass)
            obj.plt_p = plot(p.x,p.y,'ro','MarkerFaceColor','blue','DisplayName','Point Mass');
            % Waypoint
            obj.plt_wypt = plot(p.x,p.y+p.maxRad,'b*','DisplayName','new pursuit');
            % Model Trajectory
            obj.plt_traj = plot(obj.wypt_bases(:,1),obj.wypt_bases(:,2),'r--','DisplayName','gen traj');
            % Trail of robot
            obj.plt_trail = plot(p.trail.x,p.trail.y,':','DisplayName','act traj','Color','blue','LineWidth',2);
            % Add plots to running list
            obj.plts = [obj.plt_p,obj.plt_wypt,obj.plt_traj,obj.plt_trail];
            % Show circle
            if obj.show_circ
               circ_shape = obj.get_circ(p);
               obj.plt_circ = plot(circ_shape.x,circ_shape.y);
               obj.plts = [obj.plts obj.plt_circ];
            end
            % Show projected motion
            if obj.show_proj
                obj.plt_proj = plot(p.proj_mot.x,p.proj_mot.y,'r-','LineWidth',3,'DisplayName','Projected Path');
                obj.plts = [obj.plts obj.plt_proj];
            end
            % Show commanded inputs
            if obj.show_cmd
                obj.plt_cmd = quiver(p.x,p.y,1,1,'LineWidth',2,'DisplayName',sprintf("Commanded %s",p.cmd_input.name));
                obj.plts = [obj.plts obj.plt_cmd];
            end
            % Show MPC corner
            if obj.show_cornerMPC
                obj.plt_corner_mpc_r = plot([p.x,p.xc_mpc_r],[p.y,p.yc_mpc_r],'c-',"Displayname",'Right MPC corner',"LineWidth",2);
                obj.plts = [obj.plts obj.plt_corner_mpc_r];
                obj.plt_corner_mpc_l = plot([0,0],[0,0],'b-',"Displayname",'Left MPC corner',"LineWidth",2);
                obj.plts = [obj.plts obj.plt_corner_mpc_l];
            end
            % Show waypoint corner
            if obj.show_cornerWP
                obj.plt_corner_wp = plot([p.x,p.xc_wp],[p.y,p.yc_wp],'g-',"DisplayName",'Waypoint corner',"LineWidth",2);
                obj.plts = [obj.plts obj.plt_corner_wp];
            end
            % Show obstacles
            black = [0 0 0];
            for i = 1:length(obj.obss)
               obs = obj.obss{i};
               patch([obs.xs_map(1),obs.xs_map(2),obs.xs_map(2),obs.xs_map(1)],[obs.ys_map(1),obs.ys_map(1),obs.ys_map(2),obs.ys_map(2)],black);
            end
            % Formatting
            axis equal;
            xlim(obj.plt_xlim);
            ylim(obj.plt_ylim);
            % Show Legend
            if obj.show_legend
               legend(obj.plts,'Location','eastoutside'); 
            end
            % Title
            title(obj.fig_main.CurrentAxes,sprintf("MPC Motion, Time: %0.2f",p.t));
            % Record video
            if obj.rec_vid
                obj.vid = VideoWriter('vid.avi','Motion JPEG AVI');
                open(obj.vid);
            end
        end
        function initial_aux(obj,p)
            subplot(3,1,1);
            hold on;
            grid on;
            obj.plt_vxs = plot(p.ts,p.vs.vxs,'LineWidth',2);
            obj.plt_vys = plot(p.ts,p.vs.vys,'LineWidth',2);
            ylabel("Velocity (m/s)");
            legend("X vels","Y vels","Location","eastoutside");
            title("Velocities");
            subplot(3,1,2);
            hold on;
            grid on;
            obj.plt_uxs = plot(p.ts,p.cmd_inputs.x,'LineWidth',2);
            obj.plt_uys = plot(p.ts,p.cmd_inputs.y,'LineWidth',2);
            ylabel("Accelerations");
            legend("X accs", "Y accs","Location","eastoutside");
            title("Commanded Inputs");
            subplot(3,1,3);
            hold on;
            grid on;
            obj.plt_LOS = plot(p.ts,p.rs,'LineWidth',2);
            xlabel("Time (s)");
            ylabel("Line of Sight (m)");
            
        end
        function update_plot(obj,p)
            % Update robot position
            obj.plt_p.XData = p.x;
            obj.plt_p.YData = p.y;
            % Update waypoint position
            p.get_wypt(obj);
            obj.plt_wypt.XData = p.wypt.x;
            obj.plt_wypt.YData = p.wypt.y;
            % Update circle
            if obj.show_circ
                circ_shape = obj.get_circ(p);
                obj.plt_circ.XData = circ_shape.x;
                obj.plt_circ.YData = circ_shape.y;
            end
            % Update commanded inputs
            if obj.show_cmd
               obj.plt_cmd.XData = p.x;
               obj.plt_cmd.YData = p.y;
               obj.plt_cmd.UData = p.cmd_input.x;
               obj.plt_cmd.VData = p.cmd_input.y;
            end
            % Update MPC corner line
            if obj.show_cornerMPC
               obj.plt_corner_mpc_r.XData = [p.x p.xc_mpc_r];
               obj.plt_corner_mpc_r.YData = [p.y p.yc_mpc_r];
               if p.lc_active
                  obj.plt_corner_mpc_l.XData = [p.x p.xc_mpc_l];
                  obj.plt_corner_mpc_l.YData = [p.y p.yc_mpc_l];
               else
                  obj.plt_corner_mpc_l.XData = [0 0];
                  obj.plt_corner_mpc_l.YData = [0 0];
               end
            end
            % Update WP corner line
            if obj.show_cornerWP
               obj.plt_corner_wp.XData = [p.x p.xc_wp];
               obj.plt_corner_wp.YData = [p.y p.yc_wp];
            end
            % Update trail
            if obj.show_trail
               obj.plt_trail.XData = p.trail.x;
               obj.plt_trail.YData = p.trail.y;
            end
            % Update projected motion
            if obj.show_proj
                obj.plt_proj.XData = p.proj_mot.x;
                obj.plt_proj.YData = p.proj_mot.y;
            end
            % Update auxilary graphs
            obj.plt_vxs.XData = p.ts;
            obj.plt_vxs.YData = p.vs.vxs;
            obj.plt_vys.XData = p.ts;
            obj.plt_vys.YData = p.vs.vys;
            obj.plt_uxs.XData = p.ts;
            obj.plt_uxs.YData = p.cmd_inputs.x;
            obj.plt_uys.XData = p.ts;
            obj.plt_uys.YData = p.cmd_inputs.y;
            obj.plt_LOS.XData = p.ts;
            obj.plt_LOS.YData = p.rs;
            % Update title
            title(obj.fig_main.CurrentAxes,sprintf("MPC Motion, Time: %0.2f",p.t));

            if obj.rec_vid
                frame = getframe(obj.fig_main);
                writeVideo(obj.vid,frame);
            end
        end
        function close(obj)
            if obj.rec_vid
               close(obj.vid); 
            end
        end
        function postPlots(obj,p)
            vs = sqrt(p.vs.vxs.^2 + p.vs.vys.^2);
            LOSs = p.rs;
            ts = p.ts;
            subplot(2,1,1);
            plot(ts,vs);
            grid on;
            ylabel("Velocity (m/s)","LineWidth",2);
            subplot(2,1,2);
            plot(ts,LOSs);
            grid on;
            ylabel("Line of Sight (m)","LineWidth",2);
            xlabel("Time (s)");
            post_data.vs = vs;
            post_data.ts = ts;
            post_data.LOSs = LOSs;
            assignin('base','post_data',post_data);
        end
    end
end