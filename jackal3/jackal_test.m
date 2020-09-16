%% Setup ROS
% Set ros master to jackal
setenv("ROS_MASTER_URI","http://192.168.8.212:11311");
addpath('../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')
addpath('../acado_code/point_mass_export7/')
addpath('../acado_code/vel_export')

% Initialize ros in Matlab
rosinit;

global r;
r = struct("x",0.0,"y",0.0);

%%
% rossubscriber("/vicon/jackal3/jackal3","geometry_msgs/TransformStamped",@viconCB);
% 
% totaltime = 0.5;
% 
% tic;
% while toc < totaltime
%     fprintf("r pos = (%f,%f)\n",r.x,r.y);
%     pause(0.01);
% end

%% 

sub = rossubscriber("/vicon/jackal3/jackal3");
pub = rospublisher("/jackal_velocity_controller/cmd_vel","geometry_msgs/Twist");
msg = receive(sub,1);
% 
% robot = jackal_real();
% robot.init_params();

v_cmd.x = 0;
v_cmd.y = 1.0;
totaltime = 2.0;
pub_msg = rosmessage(pub);

tic;
while toc < totaltime
   msg = receive(sub,1);
   fprintf("Jackal Position (x,y): (%f,%f)\n",msg.Transform.Translation.X,msg.Transform.Translation.Y);
   pub_msg.Linear.X = 0.2;
   send(pub,pub_msg);
end
pub_msg.Linear.X = 0.0;
send(pub,pub_msg);

%% 

robot = jackal_real();
robot.init_params();
map = map_real();
map.init_params();

total_time = 20.0;
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
   robot.pub_cmd();
end