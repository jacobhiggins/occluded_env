% Run sim for point-mass
addpath(genpath('C:\Users\bezzo\Google Drive\Robotics\thinkPad_ws\Sims\acado_tests\robotsim\mathworks-robotics-mobile-robotics-simulation-toolbox-7066fa0'));
addpath('../../acado_code/point_mass_export6')
addpath('../../acado_code/point_mass_export7')
addpath('./utils')

close all;
clear all;

flags.mpc_flag = false;
flags.mpc4_flag = false;
flags.mpc5_flag = false;
flags.mpc6_flag = true;
flags.pid_flag = false;

record = true;
record_aux = true;

warning('off','MATLAB:gui:array:InvalidArrayShape');

%% Run sim
% vel_test2(record,flags); % Previous design iteration; see 06/25/2020 slides
% simulation(record,flags);
simulation2(record);