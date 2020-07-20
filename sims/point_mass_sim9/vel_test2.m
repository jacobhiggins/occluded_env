% Include obstacles, as well as new ways for waypoints/corners to generate
% based on obstacles and line of sight
function vel_test2(record,flags)
    addpath('./utils')
    %% ********* Define Constants ************
    % Set parameters of the simulation
    sysparams = sys_params();
    dt = sysparams.dt;
    v = sysparams.v;
    a = sysparams.a;
    N = sysparams.N;
    Nu = sysparams.Nu;
    start_xs = sysparams.start_xs;
    maxRange = sysparams.maxRange; % maximum range of sensor
    rs = length(start_xs); % Number of robots
    % Set map data
    m = map();
    hls = m.hls;
    hws = m.hws;
    Ms = m.Ms;
    % Initialize values
    axs = zeros(rs,1);
    ays = zeros(rs,1);
    secs = zeros(rs,1);
    xs = zeros(rs,1);
    ys = zeros(rs,1);
    xgs = zeros(rs,1);
    ygs = zeros(rs,1);
    ps = cell(rs,1);
    mpc6Inputs = cell(rs,1);
    rads = zeros(rs,1);
    for i = 1:rs
        axs(i) = 0;
        ays(i) = 0;
        secs(i) = 1;
        xs(i) = start_xs(i)*hws(1);
        ys(i) = 1;
        xgs(i) = xs(i);
        ygs(i) = ys(i)+10;
        % Initialize point mass
        p = point();
        p = p.setPos(xs(i),ys(i));
        p = p.setVel(0,0);
        p = p.setAcc(0,0);
        p = p.setMaxVel(v);
        p.dt = dt;
        ps{i} = p;
        mpc6Input.u = zeros(N,Nu+2); % Four decision variables
        mpc6Inputs{i} = mpc6Input;
        rads(i) = sysparams.rad_max;
    end
    
    %% ****** Create obss ******
    
    obss = create_obs(m);
    num_obs = length(obss);
    
    %% ****** Create Patches ******
    dmax = sysparams.rad_max;
    patches = get_patches(hls,hws);
    
    %% ****** Waypoint Bases ******

    way_pt_bases = cell(rs,1);
    trails = cell(3,1);
    for i = 1:rs
        way_pt_base = get_waypoint_base(xs(i),ys(i),hls,hws,obss);
        way_pt_bases{i} = way_pt_base;
        trail = [xs(i),ys(i)];
        trails{i} = trail;
    end
    
    %% ****** Set Corner ******
    
    xcs = zeros(rs,1);
    ycs = zeros(rs,1);
    for i = 1:rs
        [xcs(i),ycs(i)] = setCorner(0,0,xs(i),ys(i),secs(i),hws,hls,obss);
    end
    
    %% ****** Plot Simuation ******
    fig = figure(1);
    hold on;
    % Patches of uncertainty
    num_patches = patches.num;
    plt_patches = cell(1,num_patches);
    for i = 1:num_patches
        patch_xs = patches.x(i,:);
        patch_ys = patches.y(i,:);
        color = patches.color(i,:);
        plt_patch = patch(patch_xs,patch_ys,color,"EdgeColor","none");
        plt_patches{i} = plt_patch;
    end
    
    % Motion components
    plts = cell(rs,1);
    plt_goals = cell(rs,1);
    plt_goal2s = cell(rs,1);
    plt_trajs = cell(rs,1);
    plt_trails = cell(rs,1);
    plt_circs = cell(rs,1);
    cmap = colormap(lines(rs));
    for i = 1:rs
        way_pt_base = way_pt_bases{i};
        trail = trails{i};
        plt = plot(xs(i),ys(i),'ro','MarkerFaceColor',cmap(i,:),'DisplayName','Point Mass'); % point mass
%         plt_goal = plot(xgs(i),ygs(i),'r*','DisplayName','pursuit'); % pursuit
        plt_goal2 = plot(xs(i),ys(i)+rads(i),'b*','DisplayName','new pursuit');
        plt_traj = plot(way_pt_base(:,1),way_pt_base(:,2),'r--','DisplayName','gen traj'); % Generated Trajectory
        plt_trail = plot(trail(:,1),trail(:,2),':','DisplayName','act traj','Color',cmap(i,:),'LineWidth',2); % Actual trajectory
        
        plts{i} = plt;
        plt_goals{i} = plt_goals;
        plt_goal2s{i} = plt_goal2;
        plt_trajs{i} = plt_traj;
        plt_trails{i} = plt_trail;
        if sysparams.circ_on
            circ_shape = get_circ(xs(i),ys(i),rads(i));
            plt_circ = plot(circ_shape.x,circ_shape.y);
            plt_circs{i} = plt_circ;
        end
    end
    
    % Uncertainty probe
    plt_probes = cell(rs,1);
    for i = 1:rs
        way_pt_base = way_pt_bases{i};
        xm2 = way_pt_base(2,1);
        ym2 = way_pt_base(2,2);
        xm3 = way_pt_base(3,1);
        ym3 = way_pt_base(3,2);
        theta2 = atan2(ym3-ym2,xm3-xm2);
        d = norm([ym3-ym2,xm3-xm2],2);
        waypoint = set_waypoint(xs(i),ys(i),xcs(i),ycs(i),xm2,ym2,theta2,d);
        xintersect = waypoint.x;
        yintersect = waypoint.y;
        if sysparams.show_probe
            unc_probe = get_probe(xs(i),ys(i),xintersect,yintersect,theta2,dmax,patches);
            plt_probe = plot(unc_probe.x,unc_probe.y,'g-');
            plt_probes{i} = plt_probe;
        end
    end
    
    % Walls
    gray = [0.5 0.5 0.5];
    patch([hws(1),hls(2)+hws(3),hls(2)+hws(3),hws(1)],[0,0,hls(1)-hws(2),hls(1)-hws(2)],gray);
    patch([0,hls(2)-hws(3),hls(2)-hws(3),0],[hls(1),hls(1),hls(1)+hls(3)-hws(2),hls(1)+hls(3)-hws(2)],gray);
%     patch([-hws(1),-hws(1)/2,-hws(1)/2,-hws(1)],[-hls(1)-10,-hls(1)-10,hls(3),hls(3)],gray);
%     patch([hws(1)/2,hls(2) + hws(3),hls(2)+hws(3),hws(1)/2],[-hls(1)-10,-hls(1)-10,-hws(2)/2,-hws(2)/2],gray);
%     patch([-hws(1),hls(2)-hws(3)/2,hls(2)-hws(3)/2,-hws(1)],[hws(2)/2,hws(2)/2,hls(3),hls(3)],gray);
%     patch([hls(2)+hws(3)/2,hls(2)+hws(3),hls(2)+hws(3),hls(2)+hws(3)/2],[-hws(2)/2,-hws(2)/2,hls(3),hls(3)],gray);
    
    % Projection and Corner
    plt_projs = cell(rs,1);
    plt_lines = cell(rs,1);
    qvs = cell(rs,1);
    for i = 1:rs
        plt_proj = plot(xs(i),ys(i),'r-','LineWidth',3,'DisplayName','Projected Path'); % ****************REMOVE WHEN DONE TESTING
        plt_projs{i} = plt_proj;
        ann = annotation('textbox',[0.8 0.5 0.2 0.2],'String',sprintf("Cost: %f",0),'FitBoxToText','on','BackgroundColor','w');
        
        qv = quiver(xs(i),ys(i),0,0,'LineWidth',2,'DisplayName','Commanded Acc');
        qvs{i} = qv;
        plt_line = plot([xs(i),xcs(i)],[ys(i),ycs(i)]); % Line to corner
        plt_lines{i} = plt_line;
    end
    
    % Obstacles
    black = [0 0 0];
    for i = 1:num_obs
       obs = obss{i};
       x1 = -obs.w/2; % x coordinate of lower left
       y1 = -obs.h/2; % y coordinate of lower left
       x2 = obs.w/2; % upper right
       y2 = obs.h/2; % upper right
       [x1,y1] = u2c(x1,y1,obs.x,obs.y,Ms{obs.get_sec()});
       [x2,y2] = u2c(x2,y2,obs.x,obs.y,Ms{obs.get_sec()});
       wypt1 = obs.wypts{2};
       wypt2 = obs.wypts{3};
       [wypt1x,wypt1y] = u2c(wypt1(1),wypt1(2),obs.x,obs.y,Ms{obs.get_sec()});
       [wypt2x,wypt2y] = u2c(wypt2(1),wypt2(2),obs.x,obs.y,Ms{obs.get_sec()});
       
       patch([x1,x2,x2,x1],[y1,y1,y2,y2],black); % Plot obstacle
%        plot(wypt1x,wypt1y,'rx');
%        plot(wypt2x,wypt2y,'rx');
    end
    
    % Corners
    [corners,Ms_avoid,walls] = getCorners(hls,hws,obss);
%     plot(corners(:,1),corners(:,2),"o");
    
    % Formatting
    axis equal;
    xlim([0,hls(2)]);
    ylim([0,hls(3)+hls(1)-hws(2)]);
    
    if flags.pid_flag
       word = "PID Controller";
    end
    if flags.mpc_flag 
       word = "MPC Controller";
    end
    if flags.mpc4_flag
       word = "MPC Controller";
    else
       word = "MPC Controller"; 
    end
    
    if record
        vid = VideoWriter('vid.avi','Motion JPEG AVI');
        %    vid.Quality = 50;
        open(vid);
    end
    
    legend([plt,plt_goal2,qv,plt_traj,plt_trail,plt_proj],'Location','eastoutside'); %********REMOVE WHEN DONE TESTING
    
    % Plot auxillary graph
    lw = 2;
    ts_rs = cell(3,1);
    vs_rs = cell(3,1);
    as_rs = cell(3,1);
    sim_rads = cell(3,1); % Radius for waypoint over time
    plt_vxs = cell(3,1);
    plt_vys = cell(3,1);
    plt_axs = cell(3,1);
    plt_ays = cell(3,1);
    plt_rads = cell(3,1); % Plot for radius over time
    for i = 1:rs
        aux = figure(i+1);
        ts = [0];
        vs = [0;0];
        as = [0;0];
        sim_rad = [0];
        ts_rs{i} = ts;
        vs_rs{i} = vs;
        as_rs{i} = as;
        sim_rads{i} = sim_rad;
        subplot(3,1,1);
        hold on;
        plt_vx = plot(ts,vs(1,:),'LineWidth',lw);
        plt_vy = plot(ts,vs(2,:),'LineWidth',lw);
        plt_vxs{i} = plt_vx;
        plt_vys{i} = plt_vy;
        title("Velocities");
        legend("X vel","Y vel","Location",'eastoutside');
        hold off;
        subplot(3,1,2);
        hold on;
        plt_ax = plot(ts,as(1,:),'LineWidth',lw);
        plt_ay = plot(ts,as(2,:),'LineWidth',lw);
        plt_axs{i} = plt_ax;
        plt_ays{i} = plt_ay;
        legend("X acc","Y acc","Location",'eastoutside');
        title("Accelerations");
        sgtitle(sprintf("Motion for Robot %d",i));
        hold off;
        subplot(3,1,3);
        hold on;
        plt_rad = plot(ts,sim_rad,'LineWidth',lw);
%         plot([0,200],sysparams.rad_max*ones(2,1));
        plt_rads{i} = plt_rad;
        ylabel("Radius (m)");
    end
    
    %% ****** Run Simulation ******
    
    
    i = 0;
    t = 0;
    flag = true;
    trail = [];
    costs = [];
    mpc_times=[];
    dist_proj = sysparams.rad_max;
    dist_fracs = cell(rs,1);
    prev_wypts = cell(rs,1);
    for i = 1:rs
        prev_wypt.x = xs(i);
        prev_wypt.y = ys(i);
        prev_wypts{i} = prev_wypt;
    end
    while flag
        t = t + dt;
        ts = [ts t];
        for i = 1:rs
            sec = section(xs(i),ys(i),hws,hls);
            %         [xrc,yrc,flip] = setCorner(hls(2)-hws(3)/2,hls(1)+hls(3)-hws(2),x,y,sec,hws,hls,obss,Ms);
            [xcs(i),ycs(i),M_avoid,flip,c] = setCorner3(xs(i),ys(i),xcs(i),ycs(i),corners,Ms_avoid,i);% c = current corner/waypoint base
            %         [xrc,yrc,M_avoid,flip,c] = setCorner3(x,y,xrc,yrc,corners,Ms_avoid);
            wall = walls(c);
            %         if  && i<4
            %            i = i + 1;
            %         end
            way_pt_base = way_pt_bases{i};
            n = length(way_pt_base);
            xm2 = way_pt_base(min(c+1,n),1);
            ym2 = way_pt_base(min(c+1,n),2);
            xm3 = way_pt_base(min(c+2,n),1);
            ym3 = way_pt_base(min(c+2,n),2);
            %         theta2 = way_pt_base(sec+1,3);
            theta2 = atan2(ym3-ym2,xm3-xm2);
            d = norm([ym3-ym2,xm3-xm2],2);
            waypoint = set_waypoint(xs(i),ys(i),xcs(i),ycs(i),xm2,ym2,theta2,d);
            xintersect = waypoint.x; 
            yintersect = waypoint.y;
            if sec==3
                M_avoid = eye(2);
                xc = hls(2);
                yc = hls(1)+hls(3)-hws(2);
                xintersect =  hls(2)-hws(3)/2;
                yintersect = hls(3)+ hls(1);
            end
            
            %     xg = 0;%******REMOVE THESE LINES WHEN DONE
            %     yg = -hl1;
            
%             plt_goal.XData = xg;
%             plt_goal.YData = yg;

            unc_probe = get_probe(xs(i),ys(i),xintersect,yintersect,theta2,dmax,patches); % Get "uncertainty probe"
            [dist_proj,dist_fracs{i}] = exp_x(unc_probe); % expected value of x using uncertainty probe

            prev_wypt = prev_wypts{i};
            waypoint2 = set_waypoint2(xs(i),ys(i),xcs(i),ycs(i),way_pt_base,prev_wypt,rads(i),i,sysparams.rad_max,c,dist_fracs{i});
            xg2 = waypoint2.x;
            yg2 = waypoint2.y;
            prev_wypts{i} = waypoint2;
            
            p = ps{i};
            p.setCorner(xcs(i),ycs(i));
            M = Ms{sec};
            if flags.pid_flag
                [ax,ay,ex,ey] = control_PID(x,y,xg2,yg2,int_ex,int_ey,dt,ex,ey);
            end
            if flags.mpc_flag
                [vx,vy,mpc4Input] = controller_MPC(x,y,xg2,yg2,mpc4Input);
            end
            if flags.mpc4_flag
                [ax,ay,mpc4Input] = controller_MPC4(p,xg2,yg2,0,0,0,0,N,mpc4Input,M,sec,ax,ay,flip);
            end
            if flags.mpc5_flag
                [ax,ay,mpc5Input] = controller_MPC5(p,xg2,yg2,0,0,0,0,N,mpc5Input,M,sec,ax,ay,flip);
            end
            mpc6Input = mpc6Inputs{i};
            tic;
            if flags.mpc6_flag
                %            [ax,ay,mpc6Input] = controller_MPC6(p,xg,yg,delta_x,N,mpc6Input,M,sec,ax,ay);
                %            [ax,ay,mpc6Input] = controller_MPC6(p,xg,yg,dist_proj,N,mpc6Input,M_avoid,sec,ax,ay,flip,wall);
                %             [ax,ay,mpc6Input] = controller_MPC6_2(p,xg,yg,dist_proj,N,mpc6Input,M_avoid,sec,ax,ay,flip,wall);
                [ax,ay,mpc6Inputs{i}] = controller_MPC6_3(p,xg2,yg2,dist_proj,N,mpc6Input,M_avoid,sec,0,0,flip,wall); % Use dist between robot and waypoint
            end
            mpc_time = toc;
            mpc_times = [mpc_times mpc_time];
            
            % Uncertainties of patches change based on LOS
            patches = updatePatches(patches,p,xintersect,maxRange);

            
            %     if sqrt(ax^2 + ay^2) > a
            %        ax = a*ax/sqrt(ax^2 + ay^2);
            %        ay = a*ay/sqrt(ax^2 + ay^2);
            %     end
            if abs(ax)> a
                ax = sign(ax)*a;
            end
            if abs(ay) > a
                ay = sign(ay)*a;
            end
            p.updatePos();
            p.updateVel(ax,ay);
            [xs(i),ys(i)] = p.getPos();
            [vx,vy] = p.getVel();
            ps{i} = p;
            vs_rs{i} = [vs_rs{i} [vx;vy]];
            as_rs{i} = [as_rs{i} [ax;ay]];
            % Update sim plot
            plt = plts{i};
            plt.XData = xs(i);
            plt.YData = ys(i);
            [waypoint2,rads(i)] = set_waypoint2(xs(i),ys(i),xcs(i),ycs(i),way_pt_base,prev_wypt,rads(i),i,sysparams.rad_max,c,dist_fracs{i});
            sim_rad = sim_rads{i};
            sim_rad = [sim_rad rads(i)];
            sim_rads{i} = sim_rad;
            xg2 = waypoint2.x;
            yg2 = waypoint2.y;
            plt_goal2 = plt_goal2s{i};
            plt_goal2.XData = xg2;
            plt_goal2.YData = yg2;
            if sysparams.circ_on
                plt_circ = plt_circs{i};
                circ_shape = get_circ(xs(i),ys(i),rads(i));
                plt_circ.XData = circ_shape.x;
                plt_circ.YData = circ_shape.y;
            end
            qv = qvs{i};
            qv.XData = xs(i);
            qv.YData = ys(i);
            qv.UData = ax*2;
            qv.VData = ay*2;
            title(fig.CurrentAxes,sprintf("%s, t = %0.2f",word,t));
            plt_line = plt_lines{i};
            plt_line.XData = [xs(i),xcs(i)];
            plt_line.YData = [ys(i),ycs(i)];
            trail = trails{i};
            trail = [trail;[xs(i),ys(i)]];
            plt_trail = plt_trails{i};
            plt_trail.XData = trail(:,1);
            plt_trail.YData = trail(:,2);
            trails{i} = trail;
            % Update aux plot
            plt_vx = plt_vxs{i};
            plt_vy = plt_vys{i};
            plt_ax = plt_axs{i};
            plt_ay = plt_ays{i};
            plt_rad = plt_rads{i};
            vs = vs_rs{i};
            as = as_rs{i};
            plt_vx.XData = ts;
            plt_vx.YData = vs(1,:);
            plt_vy.XData = ts;
            plt_vy.YData = vs(2,:);
            plt_ax.XData = ts;
            plt_ax.YData = as(1,:);
            plt_ay.XData = ts;
            plt_ay.YData = as(2,:);
            plt_rad.XData = ts;
            plt_rad.YData = (sim_rad(:))';
            if sysparams.show_probe
                plt_probe = plt_probes{i};
                plt_probe.XData = unc_probe.x;
                plt_probe.YData = unc_probe.y;
            end
            if flags.mpc4_flag
                plt_proj.XData = mpc4Input.x(:,1);
                plt_proj.YData = mpc4Input.x(:,2);
                ann.String = sprintf("Cost: %7.0f",mpc4Input.cost);
            elseif flags.mpc5_flag
                plt_proj.XData = mpc5Input.x(:,1);
                plt_proj.YData = mpc5Input.x(:,2);
                ann.String = sprintf("Cost: %7.0f",mpc5Input.cost);
            elseif flags.mpc6_flag
                mpc6Input = mpc6Inputs{i};
                plt_proj = plt_projs{i};
                plt_proj.XData = mpc6Input.x(:,1);
                plt_proj.YData = mpc6Input.x(:,2);
%                 ann.String = sprintf("Cost: %7.0f",mpc6Input.cost);
                costs = [costs mpc6Input.cost];
            end
            % Update patch colors
            for k = 1:num_patches
               plt_patch = plt_patches{k};
               plt_patch.FaceColor = patches.color(k,:);
            end
            
            %         dist_proj = 5*int_dist(mpc6Input.x); % Path integral of projected distance
            
            %     pause(0.01);
            % y > hl3
            if ys(i) > hls(3)+hls(1)-hws(2)-5 %****************CHANGE BACK WHEN DONE TESTING
                flag = false;
            end
            
        end
        if record
            frame = getframe(fig);
            writeVideo(vid,frame);
        end
        drawnow;
    end

    assignin('base','mpc_times',mpc_times);
    assignin('base','vs_rs',vs_rs);
    if sysparams.show_vnorm
       figure(rs+2);
       subplot(2,1,1);
       hold on;
       for i = 1:rs
          vs_ri = vs_rs{i};
          vsi = sqrt((vs_ri(1,:)).^2+(vs_ri(2,:)).^2);
          plot(ts,vsi,"DisplayName",sprintf("Robot %d",i));
       end
       xlabel("time (s)");
       ylabel("velocity (m/s)");
       title("Velocity Norm Over Time");
       legend;
       subplot(2,1,2);
       hold on;
       for i = 1:rs
          sim_rad = sim_rads{i};
          plot(ts,sim_rad,"DisplayName",sprintf("Robot %d",i));
       end
       xlabel("time (s)");
       ylabel("LOS Estimate (m)");
       title("LOS Over Time");
       legend;
    end
    
    if record
        close(vid);
    end
    
end