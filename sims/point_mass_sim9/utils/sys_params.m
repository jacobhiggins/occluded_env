function sysparams = sys_params()
    % Define parameters that the system uses
    sysparams.dt = 0.1;
    sysparams.a = 5; % Max acceleration
    sysparams.v = 20; % Max velocity
    sysparams.N = 50; % Prediction Horizon
    sysparams.Nu = 2; % Number of decision variables
%     sysparams.start_xs = [0.2 0.8];
    sysparams.start_xs = [0.5];
    sysparams.circ_on = true;
    sysparams.show_probe = false;
    sysparams.rad_max = 70;
    sysparams.maxRange = 70;
    sysparams.show_vnorm = true;
end