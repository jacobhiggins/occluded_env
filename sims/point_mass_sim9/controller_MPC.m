function [vx,vy,mpcInput] = controller_MPC(xr,yr,xg,yg,mpcInput)
    mpcInput.x0 = [xr,yr];
    mpcInput.x = [xr*ones(21,1) yr*ones(21,1)];
    mpcInput.y = [xg*ones(20,1) yg*ones(20,1)];
    mpcInput.yN = [xg,yg];
    mpcOutput = acado_solver( mpcInput );
    vs = mpcOutput.u(1,:);
    mpcInput.u = mpcOutput.u;
    vx = vs(1);
    vy = vs(2);
end