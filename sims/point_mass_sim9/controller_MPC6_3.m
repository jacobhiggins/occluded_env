% Include safety objective inside mpc
% Go to waypoint, velocity regulated by safety objective
% Turn on dynamic slope constraint only when projected path passes corner

% ***New
% delta_x is the distance between the robot and the pursuit
function [ax,ay,mpc6Input,info1] = controller_MPC6_3(p,xg_in,yg_in,delta_x,N,mpc6Input,M,sec,ax_in,ay_in,flip,wall)
%     M = [flip 0;0 1]*M;
    persistent info;
    if isempty(info)
       info.var_dots = [];
    end
    scale = 1;
    left_bound = wall;
    right_bound = 0;
    if sec==3
       right_bound = 15; 
    end
%     M = [flip 0;0 1]*M;
    [xr_in,yr_in] = p.getPos();
    [vx_in,vy_in] = p.getVel();
    [ax_in,ay_in] = p.getAcc();
    [xc_in,yc_in] = p.getCorner();
    yg = yg_in;
    dt = p.dt;
    % First, transform all the coordinates
    [xr,yr] = c2u(xr_in,yr_in,xc_in,yc_in,M);
    [xg,yg] = c2u(xg_in,yg_in,xc_in,yc_in,M);
    [vx,vy] = c2u(vx_in,vy_in,0,0,M);
    [ax,ay] = c2u(ax_in,ay_in,0,0,M);
    [xc,yc] = c2u(xc_in,yc_in,xc_in,yc_in,M);

    % Visibility Objective
    m = (yc - yr)/(xc - xr);
    m_des = 0;
    phi = atan(m);
    phi_des = 0;
    phi_d = phi/sqrt(xr^2 + yr^2);
    phi_y = phi/yr;
    phi_y_des = 0;
    phi_d_des = 0;
%     m_inv = max(1/m,0);

    %*** New Stuff
    m_inv = 1/m;
    try
        x_last = mpc6Input.x(end,1); % Last (x,y) coordinate of projected motion
        y_last = mpc6Input.x(end,2);
        [x_last,y_last] = c2u(x_last,y_last,xc_in,yc_in,M);
        if y_last < 0
           m_inv = 0; 
        end
    catch
        
    end
    
    m_inv_des = -100;
    var = phi_y;
    var_des = phi_y_des;
    var_doty = 1/(xr*yr*(1+(yr/xr)^2))-atan(yr/xr)/yr^2;
    var_dotx = 1/(xr^2*(1+(yr/xr)^2));
    info.var_dots = [info.var_dots;var_doty,var_dotx];
    info1 = info;
    
    % Safety Objective
%     delta_x = 0.5*norm([xr-xg,yr-yg],2);
    safe = (vx^2 + vy^2)/(2*delta_x);
%     disp(safe);
    
%     phi = phi/d; % Uncomment for good mpc run
    mpc6Input.x0 = [xr,yr,vx,vy,xc,yc,ax,ay,var,safe,left_bound,right_bound,delta_x,m_inv];
%     xrs = (xr*ones((N+1),1) + vx*0.1*(0:N)');
%     yrs = yr*ones(N+1,1)+vy*0.1*(0:N)';
    %     mpc4Input.x = [xrs ...
%         yrs ...
    mpc6Input.x = [xr*ones((N+1),1) ...
        yr*ones((N+1),1) ...
        vx*ones(N+1,1) ...
        vy*ones(N+1,1) ...
        xc*ones((N+1),1) ...
        yc*ones((N+1),1) ...
        ax*ones(N+1,1) ...
        ay*ones(N+1,1) ...
        var*ones((N+1),1) ...
        safe*ones((N+1),1) ...
        left_bound*ones(N+1,1) ...
        right_bound*ones(N+1,1) ...
        delta_x*ones(N+1,1) ...
        m_inv*ones(N+1,1)];
%     xgs = xg*ones(N,1) + vgx*0.1*(1:N)';
%     ygs = yg*ones(N,1) + vgy*0.1*(1:N)';
    % 
    mpc6Input.y = [xg*ones(N,1) yg*ones(N,1) var_des*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1)];
    mpc6Input.yN = [xg yg var_des 0];
%     mpc4Input.y = [xgs ygs 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) var_des*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1)];
%     mpc4Input.yN = [xgs(end) ygs(end) var_des];
%     A = eye(6);
    % x, y, perception, safety, ax_dot, ay_dot, epsilon_leftwall, epsilon_safety
    A = diag([50/scale 500/scale 50000000/scale 0.001/scale 250/scale 250/scale 1000000 0.001]); % 5000000 - perc, 1000000 - safety
%     A(3,3) = 0.5;
    if sec==3
       A(3,3) = 0.000000000001; 
    end
    mpc6Input.W = repmat(A,N,1);
    mpc6Input.WN = 1*diag([A(1,1) A(2,2) A(3,3) A(4,4)]);
%     mpc3Input.W(2) = 0.000001;
    
    mpcOutput = acado_solver6( mpc6Input );
%     m_acado = mpcOutput.z(1);
%     percent_error = (m - m_acado)/m;
%     disp(percent_error)
%     mpc3Input.u = mpcOutput.u;
    as = mpcOutput.u(1,:);
%     mpc6Input.u = mpcOutput.u;
%     mpc4Input.u = zeros(50,2);
    mpc6Input.x = (inv(M)*mpcOutput.x(:,1:2)' + [xc_in*ones(N+1,1) yc_in*ones(N+1,1)]')';
    mpc6Input.cost = mpcOutput.info.objValue;
    ax_dot = as(1);
    ay_dot = as(2);
    ax = ax + ax_dot*dt;
    ay = ay + ay_dot*dt;
    [ax,ay] = c2u(ax,ay,0,0,inv(M));
%     fprintf("minv: %f\n",m_inv);
    % Finally, transform back
%     [vx,vy] = c2u(vx,vy,0,0,inv(M));
end