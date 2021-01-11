addpath('../../../acado_code/corner_test_export')

Nu = 5;
CH = 50;

x = -1;
y = -2;
xg = 2;
yg = 2;

MPC_input.u = zeros(50,Nu);
MPC_input.x0 = [x,y,0];
MPC_input.x = repmat(MPC_input.x0,CH+1,1);
MPC_input.y = repmat([xg, yg,0,0,0,0,0],CH,1);
MPC_input.yN = [xg yg];

A = diag([10,...
    10,...
    10,...
    10,...
    0.01,...
    0.01,...
    1000000000000000]);
MPC_input.W = repmat(A,CH,1);
MPC_input.WN = diag([A(1,1) A(2,2)]);

MPC_output = acado_solver_corner_test( MPC_input );
proj_motion = MPC_output.x(:,1:2);

%% Plot
close all;
plot(proj_motion(:,1),proj_motion(:,2));
axis equal;