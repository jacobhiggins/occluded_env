% Improving projected motion of MPC
% Hopefully, this leads to better motion characteristics as we
% approach corner, i.e. slow down vs keep speed
close all;
clear all;
addpath("C:\Users\bezzo\Documents\GitHub\occluded_env\sims\UGV\utils");
addpath("C:\Users\bezzo\Documents\GitHub\occluded_env\acado_code\point_mass_export7");

figure(1);
map = testMap();
map.setParams_box(3,3);

pm = point_3();
pm.initial_params(map);

map.init_plot(pm);

while ~map.sim_end
    pm.get_wypt_pursuit(map);
    pm.mpc_step3();
    pm.motion_step();
    map.update_plot(pm);
end

map.close_plot();