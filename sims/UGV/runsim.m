% Run sim UGV
addpath(genpath('C:\Users\bezzo\Google Drive\Robotics\thinkPad_ws\Sims\acado_tests\robotsim\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0'));
addpath('../../acado_code/point_mass_export6')
addpath('../../acado_code/point_mass_export7')
addpath('./utils')

close all;
clear all;

warning('off','MATLAB:gui:array:InvalidArrayShape');

%% Run sim
% simulation2(record);
sim_UGV();