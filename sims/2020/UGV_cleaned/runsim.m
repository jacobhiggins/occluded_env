% This is a cleaned version of code before paper
% Several things attempted: 
%   - using lidar + map to get waypoint, KU area
%   - re-organizing the code into more general classes


addpath('../../../acado_code/point_mass_export6')
addpath('../../../acado_code/point_mass_export7')
addpath('../../../acado_code/DD_export')
addpath('../../../acado_code/vel_export')
addpath('../../../acado_code/acc_cmd_export')
addpath('./utils')
addpath('../../../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')
% addpath(genpath('../../../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/'))

close all;
clear all;

warning('off','MATLAB:gui:array:InvalidArrayShape');

%% Run sim
% sim_UGV();
sim_cleaned();
% sim_ML();

%% Notes
% 11/25/2020
% Waypoint placement working, KU area estimation not workingS