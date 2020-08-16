% Run sim UGV
addpath('../../acado_code/point_mass_export6')
addpath('../../acado_code/point_mass_export7')
addpath('../../acado_code/DD_export')
addpath('../../acado_code/vel_export')
addpath('./utils')
addpath('../../robotsim/mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0/src/kinematics')

close all;
clear all;

warning('off','MATLAB:gui:array:InvalidArrayShape');

%% Run sim
% simulation2(record);
sim_UGV();