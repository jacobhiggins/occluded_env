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
        corners
        corners_r % Defined right corners of the map, orientation of how to avoid them
        corners_l % Defined left corners of the map
        obss % obstacles of the map
        boxs % Used in is_visible function
        patches = struct("num",10,"xstart",0.0,"xend",0.0,"ybottom",0.0,"ytop",0.0,"width",0.0,"centers",[]); % Patches of uncertainty
        end_flag = true; % true when the sim should end
        % **** Main Plots ****
        plt_p % graphic handle for point
        plt_wypt % graphic handle for waypoint
        plt_traj % graphic handle for model trajectory
        plt_trail % graphic handle for trail
        plt_proj % graphic handle for projected motion
        plt_cmd % graphic handle for commanded inputs
        plt_heading;
        plt_corner_mpc_r; % graphic handle for right corners
        plt_corner_mpc_l; % graphic handle for left corners
        plt_corner_wp;
        plts; % Running list of all plots
        plt_circ % graphic handle for waypoint circle
        plt_FOV
        plt_knownunknown % known-unknown area
        plt_patches
        plt_xlim % x limit for sim plot
        plt_ylim % y limit for sim plot
        show_trail = true; % showing trail
        show_circ = true; % boolean for showing/not showing circ
        show_FOV = true; % show FOV
        show_knownunknown = true;
        show_proj = true; % s howing/not showing projected motion
        show_cmd = true; % showing/not showing commanded inputs
        show_heading = true; % show heading
        show_cornerMPC = true; % show corner MPC
        show_cornerWP = true; % show corner waypoint
        show_patches = true;
        show_legend = true; % show legend
        % **** Auxilary Plots ****
        plt_vxs % graphics handle for x velocities
        plt_vys % graphics handle for y velocities
        plt_uxs
        plt_uys
        plt_uvs % speed cmd
        plt_uws % angular velocity command
        plt_LOS
        plt_ku_area % plot ku area
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
        function FOV = get_FOV(obj,p)
           theta = 0:0.05:2*pi;
           FOV.x = p.maxRad*cos(theta) + p.x;
           FOV.y = p.maxRad*sin(theta) + p.y;
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
            % Show patches
            if obj.show_patches
                for i = 0:(obj.patches.num-1)
                    x1 = obj.patches.xstart + i*obj.patches.width;
                    x2 = x1 + obj.patches.width;
                    y1 = obj.patches.ybottom;
                    y2 = obj.patches.ytop;
                    plt_patch = patch([x1 x2 x2 x1],[y1 y1 y2 y2],obj.patches.probs(i+1)*ones(1,3),"EdgeColor","none");
                    obj.plt_patches = cat(2,obj.plt_patches,plt_patch);
                end
            end
            % Robot (point mass)
            obj.plt_p = plot(p.x,p.y,'ro','MarkerFaceColor','blue','DisplayName','Point Mass');
            % Waypoint
            obj.plt_wypt = plot(p.x,p.y+p.maxRad,'b*','DisplayName','waypoint');
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
            % Show FOV
            if obj.show_FOV
               FOV_shape = obj.get_FOV(p);
               obj.plt_FOV = plot(FOV_shape.x,FOV_shape.y,"--","DisplayName","FOV");
               obj.plts = [obj.plts obj.plt_FOV];
            end
            % Show known unknown area
            if obj.show_knownunknown
               ku = obj.knownunknown(p);
               obj.plt_knownunknown = fill(ku.poly.x,ku.poly.y,'r',"DisplayName","Unknown Area");
               obj.plts = [obj.plts obj.plt_FOV];
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
            if obj.show_heading
               obj.plt_heading = quiver(p.x,p.y,0,1,'LineWidth',2,'DisplayName',"Heading");
               obj.plts = [obj.plts obj.plt_heading];
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
            title("Linear Velocities");
            subplot(3,1,2);
            hold on;
            grid on;
            if class(p)=="UGV"||class(p)=="turtlebot"||class(p)=="jackal"
                yyaxis left;
                obj.plt_uvs = plot(p.ts,p.cmd_inputs.vs,'LineWidth',2);
                ylabel("m/s");
                yyaxis right;
                obj.plt_uws = plot(p.ts,p.cmd_inputs.omegas,'LineWidth',2);
                legend(texlabel("v"), "\omega","Location","eastoutside");
                ylabel("rad/s");
            else
                obj.plt_uxs = plot(p.ts,p.cmd_inputs.x,'LineWidth',2);
                obj.plt_uys = plot(p.ts,p.cmd_inputs.y,'LineWidth',2);
                legend("X accs", "Y accs","Location","eastoutside");
                ylabel(p.cmd_input.name);
            end
            title("Commanded Inputs");
            subplot(3,1,3);
            hold on;
            grid on;
            yyaxis left;
            obj.plt_LOS = plot(p.ts,p.rs,'LineWidth',2);
            ylabel("m");
            yyaxis right;
            obj.plt_ku_area = plot(p.ts,p.ku.areas,'Linewidth',2);
            ylabel("m^2");
            xlabel("Time (s)");
            legend("LOS","KU Area","Location","eastoutside");
            
        end
        function update_plot(obj,p)
            
            % Update robot position
            obj.plt_p.XData = p.x;
            obj.plt_p.YData = p.y;
            % Update waypoint position
            p.get_wypt(obj);
            p.scale_wypt(obj);
            obj.plt_wypt.XData = p.wypt.x;
            obj.plt_wypt.YData = p.wypt.y;
            % Update patches
            if obj.show_patches
               for i = 1:obj.patches.num
                  plt_patch = obj.plt_patches(i);
                  plt_patch.FaceColor = obj.patches.probs(i)*ones(1,3);
               end
            end
            % Update circle
            if obj.show_circ
                circ_shape = obj.get_circ(p);
                obj.plt_circ.XData = circ_shape.x;
                obj.plt_circ.YData = circ_shape.y;
            end
            if obj.show_FOV
               FOV_shape = obj.get_FOV(p);
               obj.plt_FOV.XData = FOV_shape.x;
               obj.plt_FOV.YData = FOV_shape.y;
            end
            if obj.show_knownunknown
               ku = obj.knownunknown(p);
               try
                   obj.plt_knownunknown.XData = ku.poly.x;
                   obj.plt_knownunknown.YData = ku.poly.y;
               catch
                   obj.plt_knownunknown.XData = [-100,-99,-99,-100];
                   obj.plt_knownunknown.YData = [-100,-100,-99,-99];
               end
               p.ku.areas = [p.ku.areas ku.area];
            end
            % Update commanded inputs
            if obj.show_cmd
               obj.plt_cmd.XData = p.x;
               obj.plt_cmd.YData = p.y;
               obj.plt_cmd.UData = p.cmd_input.x;
               obj.plt_cmd.VData = p.cmd_input.y;
            end
            if obj.show_heading
               obj.plt_heading.XData = p.x;
               obj.plt_heading.YData = p.y;
               obj.plt_heading.UData = 2*cos(p.theta);
               obj.plt_heading.VData = 2*sin(p.theta);
            end
            % Update MPC corner line
            if obj.show_cornerMPC
                if p.rc_active
                    obj.plt_corner_mpc_r.XData = [p.x p.xc_mpc_r];
                    obj.plt_corner_mpc_r.YData = [p.y p.yc_mpc_r];
                else
                    obj.plt_corner_mpc_r.XData = [0 0];
                    obj.plt_corner_mpc_r.YData = [0 0];
                end
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
            if class(p)=="UGV"||class(p)=="turtlebot"||class(p)=="jackal"
                obj.plt_uvs.XData = p.ts;
                obj.plt_uvs.YData = p.cmd_inputs.vs;
                obj.plt_uws.XData = p.ts;
                obj.plt_uws.YData = p.cmd_inputs.omegas;
            else
                obj.plt_uxs.XData = p.ts;
                obj.plt_uxs.YData = p.cmd_inputs.x;
                obj.plt_uys.XData = p.ts;
                obj.plt_uys.YData = p.cmd_inputs.y;
            end
            obj.plt_LOS.XData = p.ts;
            obj.plt_LOS.YData = p.rs;
            obj.plt_ku_area.XData = p.ts;
            obj.plt_ku_area.YData = p.ku.areas;
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
        function isVis = isVisible(obj,corner,p)
            xc = corner.x;
            yc = corner.y;
            xr = p.x;
            yr = p.y;
            isVis = true;
            % Visible if not hitting obstacle
            for i = 1:length(obj.boxs)
               box = obj.boxs{i};
               [xi,yi] = polyxpoly([xr xc],[yr yc],box.x,box.y);
               if length(xi) > 1
                  isVis = false; 
               end
            end
            % Visible if within FOV range
            if norm([xr-xc,yr-yc],2) > p.maxRad
                isVis = false;
            end
        end
        function ku = knownunknown(obj,p)
            ku.area = 0;
            ku.poly.x = [-1000,-999,-999,-1000];
            ku.poly.y = [-1000,-1000,-999,-999];
            xc = p.xc_mpc_r;
            yc = p.yc_mpc_r;
           [x,y] = c2u(p.x,p.y,xc,yc,p.M_mpc);
           hw2 = abs(p.next_owall);
           
           if norm([x,y],2)>p.maxRad || y > 0.0
              return 
           end
           % Get upper wall interception point
           try
                [xus,yus] = linecirc(0,hw2,x,y,p.maxRad);
                xu = xus(1);
                yu = yus(1);
                if xus(2) > xu
                    xu = xus(2);
                    yu = yus(2);
                end
            catch
                
           end 
            
           % Get lower wall interception point
            try
                [xls,yls] = linecirc(0,0,x,y,p.maxRad);
                xl = xls(1);
                yl = yls(1);
                if xls(2) > xl
                    xl = xls(2);
                    yl = yls(2);
                end
            catch
                return
            end
            
            % Get mid point
            phi = atan2(-x,-y);
            R = (hw2 - y)/cos(phi);
            r = min(R,p.maxRad);
            xm = x + r*sin(phi);
            ym = y + r*cos(phi);
            m = p.M_mpc\[xm;ym] + [xc;yc];
            try
                u = p.M_mpc\[xu;yu] + [xc;yc];
            end
            try
                l = p.M_mpc\[xl;yl] + [xc;yc];
            end
            phi_1 = atan2(yl-y,xl-x);
            if xm < xu && ~isempty(xu)
                % Extra points for good curvature
                phi_2 = atan2(yu-y,xu-x);
%                 del_phi = phi_2 - phi_1;
                extra = p.M_mpc\([x;y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
                
                ku.poly.x = [xc;m(1);u(1);extra(1,:)';l(1)];
                ku.poly.y = [yc;m(2);u(2);extra(2,:)';l(2)];
                
            elseif ~isempty(xl)
                phi_2 = atan2(ym-y,xm-x);
                extra = p.M_mpc\([x;y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
                
                
                ku.poly.x = [xc;m(1);extra(1,:)';l(1)];
                ku.poly.y = [yc;m(2);extra(2,:)';l(2)];
            end
            ku.area = polyarea(ku.poly.x,ku.poly.y);
           
        end
        function update_probs(obj,p)
            p_z1_m1 = 0.9;
            p_z1_m0 = 1 - p_z1_m1;
            p_z0_m0 = 0.9;
            p_z0_m1 = 1 - p_z0_m0;
            p_move = 0.5;
            p_stay = 0.5;
            % Update probs based on measurement
            for i = 1:obj.patches.num
               if norm([p.x-obj.patches.centers(1,i),p.y-obj.patches.centers(2,i)]) < p.r
                  obj.patches.probs(i) = p_z0_m0*obj.patches.probs(i)/(p_z0_m0*obj.patches.probs(i) + p_z0_m1*(1-obj.patches.probs(i)));
               end
            end
            % Project into the future
            if abs(obj.patches.centers(2,1)-p.y) > obj.hws(2)/2 % If far enough away
                for t = p.dt:p.dt:max(obj.patches.centers(2,1)-p.y,0)/p.vy
                    for i = 1:obj.patches.num-1
                        obj.patches.probs(i) = p_move*obj.patches.probs(i+1) + p_stay*obj.patches.probs(i);
                    end
                end
            end
        end
    end
end