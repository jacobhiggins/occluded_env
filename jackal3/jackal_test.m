%% Setup environment
% Set ros master to jackal

addpath('../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')
addpath('../acado_code/point_mass_export7/')
addpath('../acado_code/vel_export')

% Initialize ros in Matlab
try
    setenv("ROS_MASTER_URI","http://192.168.8.212:11311");
    rosinit;
catch
    disp("ROS already initialized....");
end
%% 

manual_drive = false;

robot = jackal_real();
robot.init_params();
map = map_real();
map.init_params();

total_time = 16.0;
tic;
while toc < total_time
   % Get position of jackal
   robot.get_pose();
   % Get new waypoint position
   robot.get_wypt(map);
   % Find commanded mpc inputs
   robot.mpc_stepvel();
   % Translate MPC cmd to jackal cmds
   robot.cmd_step();
   % Publish commands
   if ~manual_drive
       robot.pub_cmd();
   end
end

save("robot.mat","robot");