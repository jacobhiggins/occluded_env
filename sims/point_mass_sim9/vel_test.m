function vel_test(record,flags)

    addpath('../point_mass_export4')
    addpath('./utils')
    %% ********* Define Constants ************
    % Set parameters of the simulation
    sysparams = sys_params();
    dt = sysparams.dt;
    v = sysparams.v;
    a = sysparams.a;
    N = sysparams.N;
    Nu = sysparams.Nu;
    % Set map data
    m = map();
    hls = m.hls;
    hws = m.hws;
    Ms = m.Ms;
    % Initialize values
    v1 = 10;
    v2 = 10;
    v3 = 10;
    ax1 = 0;
    ay1 = 0;
    ax2 = 0;
    ay2 = 0;
    ax3 = 0;
    ay3 = 0;
    sec1 = 1;
    sec2 = 1;
    sec3 = 1;
    rad1 = 50;
    rad2 = 50;
    rad3 = 50;
    x1 = hws(1)/5;
    y1 = 1;
    x2 = hws(1)/2;
    y2 = 1;
    x3 = 4*hws(1)/5;
    y3 = 1;
    xg1 = x1;
    yg1 = y1+rad1;
    xg2 = x2;
    yg2 = y2+rad2;
    xg3 = x3;
    yg3 = y3+rad3;
    % PID Inputs
    int_ex = 0;
    int_ey = 0;
    ex = 0;
    ey = 0;
    % Initialize point mass
    p1 = point();
    p1 = p1.setPos(x1,y1);
    p1 = p1.setVel(0,0);
    p1 = p1.setMaxVel(v);
    p1.dt = dt;
    p2 = point();
    p2 = p2.setPos(x2,y2);
    p2 = p2.setVel(0,0);
    p2 = p2.setMaxVel(v);
    p2.dt = dt;
    p3 = point();
    p3 = p3.setPos(x3,y3);
    p3 = p3.setVel(0,0);
    p3 = p3.setMaxVel(v);
    p3.dt = dt;
    % Set corner
    mpc4Input.u = zeros(N,Nu);
    mpc5Input.u = zeros(N,Nu+1);
    
    %% ****** Generating trajectory ******
    way_pts1 = [x1,y1;x1,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(3)+ hls(1)];
    way_pts2 = [x2,y2;x2,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(3)+ hls(1)];
    way_pts3 = [x3,y3;x3,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(1)-hws(2)/2;hls(2)-hws(3)/2,hls(3)+ hls(1)];
    [ xgs1, ygs1, vxgs, vygs, axgs, aygs ] = traj_gen(way_pts1,v,dt);
    [ xgs2, ygs2, vxgs, vygs, axgs, aygs ] = traj_gen(way_pts2,v,dt);
    [ xgs3, ygs3, vxgs, vygs, axgs, aygs ] = traj_gen(way_pts3,v,dt);
    trail1 = [x1,y1];
    trail2 = [x2,y2];
    trail3 = [x3,y3];
    
    %% ****** Create obss ******
    
    obss = create_obs(m);
    num_obs = length(obss);
    
    %% ****** Set Corner ******
    
    [xrc1,yrc1] = setCorner(0,0,x1,y1,sec1,hws,hls,obss);
    [xrc2,yrc2] = setCorner(0,0,x2,y2,sec2,hws,hls,obss);
    [xrc3,yrc3] = setCorner(0,0,x3,y3,sec3,hws,hls,obss);
    
    %% ****** Plot Simuation ******
    fig = figure(1);
    hold on;
    % Motion components
    plt1 = plot(x1,y1,'ro','MarkerFaceColor','green','DisplayName','Point Mass'); % point mass
    plt_goal1 = plot(xgs1(1),ygs1(1),'r*','DisplayName','pursuit'); % pursuit
    plt_traj1 = plot(xgs1,ygs1,'r:','DisplayName','gen traj'); % Generated Trajectory
    plt_trail1 = plot(trail1(:,1),trail1(:,2),'b:','DisplayName','act traj'); % Actual trajectory
    
    plt2 = plot(x2,y2,'ro','MarkerFaceColor','green','DisplayName','Point Mass'); % point mass
    plt_goal2 = plot(xgs2(1),ygs2(1),'r*','DisplayName','pursuit'); % pursuit
    plt_traj2 = plot(xgs2,ygs2,'r:','DisplayName','gen traj'); % Generated Trajectory
    plt_trail2 = plot(trail2(:,1),trail2(:,2),'b:','DisplayName','act traj'); % Actual trajectory
    
    plt3 = plot(x3,y3,'ro','MarkerFaceColor','green','DisplayName','Point Mass'); % point mass
    plt_goal3 = plot(xgs3(1),ygs3(1),'r*','DisplayName','pursuit'); % pursuit
    plt_traj3 = plot(xgs3,ygs3,'r:','DisplayName','gen traj'); % Generated Trajectory
    plt_trail3 = plot(trail3(:,1),trail3(:,2),'b:','DisplayName','act traj'); % Actual trajectory
    
    % Walls
    gray = [0.5 0.5 0.5];
    patch([hws(1),hls(2)+hws(3),hls(2)+hws(3),hws(1)],[0,0,hls(1)-hws(2),hls(1)-hws(2)],gray);
    patch([0,hls(2)-hws(3),hls(2)-hws(3),0],[hls(1),hls(1),hls(1)+hls(3)-hws(2),hls(1)+hls(3)-hws(2)],gray);
%     patch([-hws(1),-hws(1)/2,-hws(1)/2,-hws(1)],[-hls(1)-10,-hls(1)-10,hls(3),hls(3)],gray);
%     patch([hws(1)/2,hls(2) + hws(3),hls(2)+hws(3),hws(1)/2],[-hls(1)-10,-hls(1)-10,-hws(2)/2,-hws(2)/2],gray);
%     patch([-hws(1),hls(2)-hws(3)/2,hls(2)-hws(3)/2,-hws(1)],[hws(2)/2,hws(2)/2,hls(3),hls(3)],gray);
%     patch([hls(2)+hws(3)/2,hls(2)+hws(3),hls(2)+hws(3),hls(2)+hws(3)/2],[-hws(2)/2,-hws(2)/2,hls(3),hls(3)],gray);
    
%     plt_proj = plot(x2,y2,'r-','LineWidth',3,'DisplayName','Projected Path'); % ****************REMOVE WHEN DONE TESTING
%     ann = annotation('textbox',[0.8 0.5 0.2 0.2],'String',sprintf("Cost: %f",0),'FitBoxToText','on','BackgroundColor','w');
    
    qv1 = quiver(x1,y1,0,0,'LineWidth',2,'DisplayName','Commanded Acc');
    qv2 = quiver(x2,y2,0,0,'LineWidth',2,'DisplayName','Commanded Acc');
    qv3 = quiver(x3,y3,0,0,'LineWidth',2,'DisplayName','Commanded Acc');
%     plt_rline1 = plot([x1,xrc1],[y1,yrc1]); % Line to corner
%     plt_rline2 = plot([x2,xrc2],[y2,yrc2]); % Line to corner
%     plt_rline3 = plot([x3,xrc3],[y3,yrc3]); % Line to corner
    
    % Obstacles
    black = [0 0 0];
    for i = 1:num_obs
       obs = obss{i};
       obsx1 = -obs.w/2; % x coordinate of lower left
       obsy1 = -obs.h/2; % y coordinate of lower left
       obsx2 = obs.w/2; % upper right
       obsy2 = obs.h/2; % upper right
       [obsx1,obsy1] = u2c(obsx1,obsy1,obs.x,obs.y,Ms{obs.get_sec()});
       [obsx2,obsy2] = u2c(obsx2,obsy2,obs.x,obs.y,Ms{obs.get_sec()});
       
  
       patch([x1,x2,x2,x1],[y1,y1,y2,y2],black);
    end
    
    % Formatting
    axis equal;
%     xlim([0,hls(2)]);
%     ylim([0,hls(3)+hls(1)-hws(2)]);
    xlim([0,hws(1)+10]);
    ylim([0,hls(1)+10]);
    
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
    
    legend([plt2,plt_goal2,qv2,plt_traj2,plt_trail2],'Location','eastoutside'); %********REMOVE WHEN DONE TESTING
    
    % Plot auxillary graph
    lw = 2;
    aux = figure(2);
    ts = [0];
    v1s = [0];
    v2s = [0];
    v3s = [0];
    as = [0;0];
    hold on;
    plt_v1 = plot(ts,v1s(1,:),'LineWidth',lw,'DisplayName',"Left PM");
    plt_v2 = plot(ts,v2s(1,:),'LineWidth',lw,'DisplayName','Mid PM');
    plt_v3 = plot(ts,v3s(1,:),'LineWidth',lw,'DisplayName','Right PM');
    xlabel("time (s)");
    ylabel("Reference Velocity (m/s)");
    legend('Location','eastoutside');
%     subplot(2,1,1);
%     hold on;
%     plt_vx = plot(ts,vs(1,:),'LineWidth',lw);
%     plt_vy = plot(ts,vs(2,:),'LineWidth',lw);
%     legend("X vel","Y vel","Location",'eastoutside');
%     hold off;
%     subplot(2,1,2);
%     hold on;
%     plt_ax = plot(ts,as(1,:),'LineWidth',lw);
%     plt_ay = plot(ts,as(2,:),'LineWidth',lw);
%     legend("X acc","Y acc","Location",'eastoutside');
%     hold off;
    
    %% ****** Run Simulation ******
    
    
    i = 0;
    t = 0;
    flag = true;
    trail2 = [];
    while flag
        t = t + dt;
        ts = [ts t];
    %     [xg,yg,vxg,vyg] = trajectory_line(x,y,dt,5,xg,yg,hl2);
%         try
%         i = i + 1;
%         xg = xgs(i);
%         yg = ygs(i);
%         vxg = vxgs(i);
%         vyg = vygs(i);
%         axg = axgs(i);
%         ayg = aygs(i);
%         catch e
%             xg = xgs(end);
%             yg = ygs(end);
%             vxg = vxgs(end);
%             vyg = vygs(end);
%             axg = axgs(end);
%             ayg = aygs(end);
%         end
        [xg1,yg1,vxg1,vyg1,v1,v_set1] = trajectory_line3(x1,y1,dt,v1,xrc1,yrc1,xg1,yg1,hls,hws,way_pts1,Ms{sec1},sec1);
        disp(v3);
        [xg2,yg2,vxg2,vyg2,v2,v_set2] = trajectory_line3(x2,y2,dt,v2,xrc2,yrc2,xg2,yg2,hls,hws,way_pts2,Ms{sec2},sec2);
        [xg3,yg3,vxg3,vyg3,v3,v_set3] = trajectory_line3(x3,y3,dt,v3,xrc3,yrc3,xg3,yg3,hls,hws,way_pts3,Ms{sec3},sec3);
%         disp(v3);

    %     xg = 0;%******REMOVE THESE LINES WHEN DONE
    %     yg = -hl1;

        plt_goal1.XData = xg1;
        plt_goal1.YData = yg1;
        plt_goal2.XData = xg2;
        plt_goal2.YData = yg2;
        plt_goal3.XData = xg3;
        plt_goal3.YData = yg3;
        sec1 = section(x1,y1,hws,hls);
        sec2 = section(x2,y2,hws,hls);
        sec3 = section(x3,y3,hws,hls);
        [xrc1,yrc1,flip] = setCorner(xg1,yg1,x1,y1,sec1,hws,hls,obss,Ms);
        [xrc2,yrc2,flip] = setCorner(xg2,yg2,x2,y2,sec2,hws,hls,obss,Ms);
        [xrc3,yrc3,flip] = setCorner(xg3,yg3,x3,y3,sec3,hws,hls,obss,Ms);
        p1.setCorner(xrc1,yrc1);
        p2.setCorner(xrc2,yrc2);
        p3.setCorner(xrc3,yrc3);
        M1 = Ms{sec1};
        M2 = Ms{sec2};
        M3 = Ms{sec3};
        if flags.pid_flag
            [ax2,ay2,ex,ey] = control_PID(x2,y2,xg2,yg2,int_ex,int_ey,dt,ex,ey);
        end
        if flags.mpc_flag
            [vx,vy,mpc4Input] = controller_MPC(x2,y2,xg2,yg2,mpc4Input);
        end
        if flags.mpc4_flag
           [ax2,ay2,mpc4Input] = controller_MPC4(p2,xg2,yg2,0,0,0,0,N,mpc4Input,M2,sec2,ax2,ay2,flip); 
        end
        if flags.mpc5_flag
           [ax1,ay1,mpc5Input] = controller_MPC5(p1,xg1,yg1,vxg1,vyg1,0,0,N,mpc5Input,M1,sec1,ax1,ay1,flip);
           [ax2,ay2,mpc5Input] = controller_MPC5(p2,xg2,yg2,vxg2,vyg2,0,0,N,mpc5Input,M2,sec2,ax2,ay2,flip);
           [ax3,ay3,mpc5Input] = controller_MPC5(p3,xg3,yg3,vxg3,vyg3,0,0,N,mpc5Input,M3,sec3,ax3,ay3,flip);
        end
    %     if sqrt(ax^2 + ay^2) > a
    %        ax = a*ax/sqrt(ax^2 + ay^2);
    %        ay = a*ay/sqrt(ax^2 + ay^2);
    %     end
        if abs(ax1)> a
           ax1 = sign(ax1)*a;
        end
        if abs(ay1) > a
           ay1 = sign(ay1)*a;
        end
        if abs(ax2)> a
           ax2 = sign(ax2)*a;
        end
        if abs(ay2) > a
           ay2 = sign(ay2)*a;
        end
        if abs(ax3)> a
           ax3 = sign(ax3)*a;
        end
        if abs(ay3) > a
           ay3 = sign(ay3)*a;
        end
        p1.updatePos();
        p1.updateVel(ax1,ay1);
        p2.updatePos();
        p2.updateVel(ax2,ay2);
        p3.updatePos();
        p3.updateVel(ax3,ay3);
        [x1,y1] = p1.getPos();
        [x2,y2] = p2.getPos();
        [x3,y3] = p3.getPos();
        [vx1,vy1] = p1.getVel();
        [vx2,vy2] = p2.getVel();
        [vx3,vy3] = p3.getVel();
        v1s = [v1s norm([vx1,vy1],2)];
        v2s = [v2s norm([vx2,vy2],2)];
        v3s = [v3s norm([vx3,vy3],2)];
%         [vx,vy] = p2.getVel();
%         vs = [vs [vx;vy]];
        as = [as [ax2;ay2]];
        % Update sim plot
        plt1.XData = x1;
        plt1.YData = y1;
        plt2.XData = x2;
        plt2.YData = y2;
        plt3.XData = x3;
        plt3.YData = y3;
        qv1.XData = x1;
        qv1.YData = y1;
        qv2.XData = x2;
        qv2.YData = y2;
        qv3.XData = x3;
        qv3.YData = y3;
        qv1.UData = ax1*2;
        qv1.VData = ay1*2;
        qv2.UData = ax2*2;
        qv2.VData = ay2*2;
        qv3.UData = ax3*2;
        qv3.VData = ay3*2;
        title(fig.CurrentAxes,sprintf("%s, t = %0.2f",word,t));
%         plt_rline1.XData = [x1,xrc1];
%         plt_rline1.YData = [y1,yrc1];
%         plt_rline2.XData = [x2,xrc2];
%         plt_rline2.YData = [y2,yrc2];
%         plt_rline2.XData = [x2,xrc2];
%         plt_rline2.YData = [y2,yrc2];
        trail1 = [trail1;[x1,y1]];
        trail2 = [trail2;[x2,y2]];
        trail3 = [trail3;[x3,y3]];
        plt_trail1.XData = trail1(:,1);
        plt_trail1.YData = trail1(:,2);
        plt_trail2.XData = trail2(:,1);
        plt_trail2.YData = trail2(:,2);
        plt_trail3.XData = trail3(:,1);
        plt_trail3.YData = trail3(:,2);
        % Update aux plot
        plt_v1.XData = ts;
        plt_v1.YData = v1s;
        plt_v2.XData = ts;
        plt_v2.YData = v2s;
        plt_v3.XData = ts;
        plt_v3.YData = v3s;
%         plt_vx.XData = ts;
%         plt_vx.YData = vs(1,:);
%         plt_vy.XData = ts;
%         plt_vy.YData = vs(2,:);
%         plt_ax.XData = ts;
%         plt_ax.YData = as(1,:);
%         plt_ay.XData = ts;
%         plt_ay.YData = as(2,:);
        if flags.mpc4_flag
            plt_proj.XData = mpc4Input.x(:,1);
            plt_proj.YData = mpc4Input.x(:,2);
            ann.String = sprintf("Cost: %7.0f",mpc4Input.cost);
        elseif flags.mpc5_flag
            plt_proj.XData = mpc5Input.x(:,1);
            plt_proj.YData = mpc5Input.x(:,2);
            ann.String = sprintf("Cost: %7.0f",mpc5Input.cost);
        end
        
        drawnow;
    %     pause(0.01);
        % y > hl3
        if x2 > hws(1) + 10 %****************CHANGE BACK WHEN DONE TESTING
           flag = false; 
        end
        if record
            frame = getframe(fig);
            writeVideo(vid,frame);
        end
    end

    if record
        close(vid);
    end

end