% Returns a series of hallway dimensions and linear transformation 
% matrices that describe the look of the map
function map = map1()
    addpath("./utils");
    sysparams = sys_params();
    res = 2; % resolution of occupancy grid, square per meter
    hl1 = 100; % Hall length
    hw1 = 30; % Hall width
    M1 = eye(2); % linear transformation
    
    hl2 = 200;
    hw2 = 30;
    M2 = [0 1; 1 0];
    
    hl3 = 100;
    hw3 = 30;
    M3 = eye(2);
    
    map.hws = [hw1,hw2,hw3];
    map.hls = [hl1,hl2,hl3];
    map.Ms = {M1,M2,M3};
    
    map.obss = create_obs(map);
    map.patches = get_patches(map.hls,map.hws);
    map.start_pose.x = sysparams.start_xs(1)*hw1;
    map.start_pose.y = 1;
    map.wypt_bases = get_waypoint_base(map.start_pose.x,map.start_pose.y,map.hls,map.hws,map.obss);
    
%      0 0 0 0 0 0 0 0 0 0 1 1 1 1 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      0 0 0 0 0 0 0 0 0 0 1 0 0 0 1
%      1 1 1 1 1 1 1 1 1 1 1 0 0 0 1
%      1 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%      1 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%      1 0 0 0 0 0 0 0 0 0 0 0 0 0 1
%      1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 0 0 0 1 0 0 0 0 0 0 0 0 0 0
%      1 1 1 1 1 0 0 0 0 0 0 0 0 0 0
    
    
    p = zeros((hl1+hl3-hw2)*res + 2,hl2*res + 2); % Set stage
    p(end-(hl1)*res-1:end,1)=1; % left vertical wall, hlwy 1
    p(end,1:hw1*res+2)=1; % Start horizontal wall, hlwy 1
    p(end-(hl1-hw2)*res:end,hw1*res+2)=1; % right vertical wall, hlwy 1
    p(end-(hl1)*res-1,1:(hl2-hw3)*res + 1)=1; % top wall, hlwy 2
    p(end-(hl1-hw2)*res,hw1*res+2:end)=1; % bottom wall, hlwy 2
    p(1:end-(hl1)*res-1,(hl2-hw3)*res + 1)=1; % left vertical wall, hlwy 3
    p(1:end-(hl1-hw2)*res,end)=1; % right vertical hallway, hlwy 3
    p(1,(hl2-hw3)*res + 1:end)=1; % End horizontal wall, hlwy 3
    
    map.res = res;
    map.p = p;
end