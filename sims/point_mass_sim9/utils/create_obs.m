function obss = create_obs(map)
    addpath('./utils')
    hws = map.hws;
    hls = map.hls;
    Ms = map.Ms;
    obs_secs = [];
%     obs_secs = [2,2]; % Define section for obstacles
    obs_ls = [30,30]; % Define lengths of obstacles
    obs_ws = [15,15]; % Define widths of obstacles
%     fracws = [1];
%     fracls = [0.5];
    fracws = [1,0];
    fracls = [0.5,0.1];
    num_obs = length(obs_secs);
    obss = {};
    for i = 1:num_obs
       sec = obs_secs(i);
       M = Ms{sec};
       obs_l = obs_ls(i);
       obs_w = obs_ws(i);
       fracw = fracws(i);
       fracl = fracls(i);
       [obs_xc,obs_yc] = setCorner(0,0,0,0,sec,hws,hls,{},{});
       o = obs(); % NOTE: Can't name instance of object the name of object type
       o.set_sec(sec);
       o.set_dims(obs_w,obs_l);
       o.set_pos(obs_xc,obs_yc,M,hws(sec),hls(sec),fracw,fracl);
       o.set_wypts(hws(sec));
%        o.set_avoid(hws(sec));
       obss{i} = o;
    end
end