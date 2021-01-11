load exampleMaps.mat
% map = binaryOccupancyMap(simpleMap,2);
map = office_area_map;
show(map);
robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated);
prm  = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 50;
prm.ConnectionDistance = 5;
startLoc = [2 1];
endLoc = [12 10];
path = findpath(prm, startLoc, endLoc);
show(prm);