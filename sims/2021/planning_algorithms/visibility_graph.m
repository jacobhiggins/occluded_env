%%
vrm = true;
prm = false;

addpath("./algorithms");
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);
figure(1);
hold on;
show(map);
robotRadius = 0.0;
% mapInflated = copy(map);
% inflate(mapInflated,robotRadius);
% show(mapInflated);

%% Visibility
if vrm
    vis = visibility(map);
    vis.get_corners();
    scatter(vis.corners(:,1),vis.corners(:,2))
    vis.make_roadmap();
    start = [1,1];
    goal = [12,8];
    vis.find_path(start,goal);
end
%% Prm
if prm
    prm  = mobileRobotPRM;
    prm.Map = map;
    prm.NumNodes = 500;
    prm.ConnectionDistance = 5;
    startLoc = [1 1];
    endLoc = [24 19];
    path = findpath(prm, startLoc, endLoc);
    show(prm);
end
