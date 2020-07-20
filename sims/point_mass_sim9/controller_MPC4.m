function [ax,ay,mpc4Input] = controller_MPC4(p,xg_in,yg_in,vgx_in,vgy_in,axg_in,ayg_in,N,mpc4Input,M,sec,ax_in,ay_in,flip)
    scale = 1;
    left_bound = -22.5;
    M = [flip 0;0 1]*M;
    [xr_in,yr_in] = p.getPos();
    [vx_in,vy_in] = p.getVel();
    [xc_in,yc_in] = p.getCorner();
    yg = yg_in;
    dt = p.dt;
    % First, transform all the coordinates
    [xr,yr] = c2u(xr_in,yr_in,xc_in,yc_in,M);
    [xg,yg] = c2u(xg_in,yg_in,xc_in,yc_in,M);
    [vgx,vgy] = c2u(vgx_in,vgy_in,0,0,M);
    [agx,agy] = c2u(axg_in,ayg_in,0,0,M);
    [vx,vy] = c2u(vx_in,vy_in,0,0,M);
    [ax,ay] = c2u(ax_in,ay_in,0,0,M);
    [xc,yc] = c2u(xc_in,yc_in,xc_in,yc_in,M);
    % Next, get inputs from MPC
    m = (yc - yr)/(xc - xr);
    m_des = 0;
    phi = atan(m);
    phi_des = 0;
    phi_d = phi/sqrt(xr^2 + yr^2);
    phi_y = phi/yr;
    phi_y_des = 0;
    phi_d_des = 0;
    m_inv = -1/m;
    m_inv_des = -100;
    var = phi_y;
    var_des = phi_y_des;
%     phi = phi/d; % Uncomment for good mpc run
    mpc4Input.x0 = [xr,yr,vx,vy,xc,yc,ax,ay,var,left_bound];
    mpc4Input.x = [xr*ones((N+1),1) ...
        yr*ones((N+1),1) ...
        vx*ones(N+1,1) ...
        vy*ones(N+1,1) ...
        xc*ones((N+1),1) ...
        yc*ones((N+1),1) ...
        ax*ones(N+1,1) ...
        ay*ones(N+1,1) ...
        var*ones((N+1),1) ...
        left_bound*ones(N+1,1)];
    mpc4Input.y = [xg*ones(N,1) yg*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) 0*ones(N,1) var_des*ones(N,1)];
    mpc4Input.yN = [xg yg var_des];
%     A = eye(6);
    A = diag([10/scale 100/scale 0.1/scale 0.1/scale 1/scale 1/scale 1/scale 1/scale 5000000/scale]); % 2000000
%     A(3,3) = 0.5;
    if sec==3
       A(9,9) = 0.000000000001; 
    end
    mpc4Input.W = repmat(A,N,1);
    mpc4Input.WN = 1*diag([A(1,1) A(2,2) A(9,9)]);
%     mpc3Input.W(2) = 0.000001;
    mpcOutput = acado_solver4( mpc4Input );
%     m_acado = mpcOutput.z(1);
%     percent_error = (m - m_acado)/m;
%     disp(percent_error)
%     mpc3Input.u = mpcOutput.u;
    as = mpcOutput.u(1,:);
%     mpc4Input.u = mpcOutput.u;
    mpc4Input.u = zeros(50,2);
    mpc4Input.x = (inv(M)*mpcOutput.x(:,1:2)' + [xc_in*ones(N+1,1) yc_in*ones(N+1,1)]')';
    mpc4Input.cost = mpcOutput.info.objValue;
    ax_dot = as(1);
    ay_dot = as(2);
    ax = ax + ax_dot*dt;
    ay = ay + ay_dot*dt;
    [ax,ay] = c2u(ax,ay,0,0,inv(M));
%     fprintf("minv: %f\n",m_inv);
    % Finally, transform back
%     [vx,vy] = c2u(vx,vy,0,0,inv(M));
end