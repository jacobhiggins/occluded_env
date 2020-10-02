%% Setup environment
% Set ros master to jackal

clear all;

warning('off','ros:mlros:common:SavedObjectInvalid')

addpath('../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')
addpath('../acado_code/point_mass_export7/')
addpath('../acado_code/vel_export')
addpath('./acado_code_jackal3/vel_jackal3_export');

% Initialize ros in Matlab
try
    setenv("ROS_MASTER_URI","http://192.168.8.212:11311");
    rosinit;
catch
    disp("ROS already initialized....");
end
%% 

manual_drive = true;

robot = jackal_real();
robot.init_params();
map = map_real();
map.init_params(robot);
vis = visualizer();
vis.init(robot,map);

tic; % Global time for all components
toc_prev = toc;
freqs = [];
while ~map.stop && toc < 10
   freqs = [freqs 1/(toc-toc_prev)];
   toc_prev = toc;
%     pub_msg = rosmessage(robot.pub);
%     pub_msg.Linear.X = 0.1;
%     pub_msg.Angular.Z = 0.0;
%     send(robot.pub,pub_msg);
   % Check stop experiment
%    map.check_stop_experiment(robot);
   % Get position of jackal
   robot.get_pose();
   % Get corners
   robot.get_corner(map);
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
   vis.update(robot,map);
end

if ~manual_drive
    robot.stop();
end

vis.close();

save("robot_experiment.mat","robot","map");