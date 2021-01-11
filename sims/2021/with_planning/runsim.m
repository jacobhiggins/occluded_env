% Run sim UGV
addpath('./maps')
addpath('./algorithms')
addpath('./simulations')
addpath('../../../acado_code/point_mass_export6')
addpath('../../../acado_code/point_mass_export7')
addpath('../../../acado_code/DD_export')
addpath('../../../acado_code/vel_export')
addpath('../../../acado_code/acc_cmd_export')
addpath('../../../acado_code/acc_cmd_local_frame_export')
addpath('./utils')
addpath('../../../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')
% addpath(genpath('../../../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/'))

close all;
clear all;

warning('off','MATLAB:gui:array:InvalidArrayShape');

%% Run sim
% sim_UGV();
% sim_cleaned();
% sim_ML();
sim_w_binary();

%% Notes
% 11/25/2020
% Changed frame of MPC to frame on reference trajectory, i.e. everything is
% constantly being translated and rotated if robot is in motion

% 12/02/2020
% Recording data for eventual ML training, making sure everything looks
% okay
% Changes just for ML:
% - When lidar senses wall, can only detect corner wall (speed)
% - Second hall width = 5, but ref traj is 3.5 m from top (more accurate KU
% area calculation since the KU area with lidar is still lacking)

% 12/17/2020
% Added left/right/top constraints to MPC
% Added noise (see StateFilter object)
% Added ability to infer sections of freespace with lidar (given a set of
% lidar points, between which lidar points is free space?)
% Added ability of algorithm to place waypoint dependent on if the
% edge-of-FOV waypoint is occluded or not
