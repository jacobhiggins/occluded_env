function [waypoint,d] = set_waypoint2(x,y,xc,yc,way_pt_base,wypt_prev,d,pm_index,rad_max,c,dist_frac)
    persistent dists; % Distances between waypoints
    persistent vecs; % vectors between waypoints
    persistent cs; % current waypoint base
    if isempty(dists)
       for i = 1:length(way_pt_base)-1
           wypta = way_pt_base(i,:);
           wyptb = way_pt_base(i+1,:);
           vec = wyptb - wypta;
           vec = vec/norm(vec,2);
           dists = [dists;norm(wypta-wyptb,2)];
           vecs = [vecs;vec];
       end
    end
    if isempty(cs)
       cs = ones(3,1);
    end
    cs(pm_index) = min(length(dists),cs(pm_index)); % current waypoint index for circle
    wpb = way_pt_base(cs(pm_index),:); % current waypoint index for radius
    next_wpb = way_pt_base(min(cs(pm_index)+1,size(way_pt_base,1)),:); % next waypoint for circle
    next_next_wpb = way_pt_base(min(cs(pm_index)+2,size(way_pt_base,1)),:); % next next waypoint for circle
    vec = vecs(cs(pm_index),:); % vector between current and next waypoint for circle
    m = vec(2)/vec(1); % slope of line bewteen current and next waypoint for circle
    
    % Determination of radius using appropriate waypoints
    n = length(way_pt_base);
    xm2 = way_pt_base(min(c+1,n),1);
    ym2 = way_pt_base(min(c+1,n),2);
    xm3 = way_pt_base(min(c+2,n),1);
    ym3 = way_pt_base(min(c+2,n),2);
    theta2 = atan2(ym3-ym2,xm3-xm2);
    
    d = set_radius(x,y,xc,yc,xm2,ym2,theta2,d,rad_max,c,1);
    [xis,yis] = linecirc(m,0,x-wpb(1),y-wpb(2),d); % x's and y's where circle intersects line
    tmp1 = -Inf;
    % Pick point of intersection that has the highest dot product with vec
    for i = 1:2
       tmp = vec(1)*xis(i) + vec(2)*yis(i);
       if tmp > tmp1
          xi = xis(i);
          yi = yis(i);
          tmp1 = tmp;
       end
    end
    try
        waypoint.x = xi + wpb(1);
        waypoint.y = yi + wpb(2);
        vec1 = [waypoint.x-wypt_prev.x;waypoint.y-wypt_prev.y];
    catch
        vec1 = -vec';
    end
    if vec*vec1 < 0
        waypoint = wypt_prev;
    end
    if norm([x-next_wpb(1),y-next_wpb(2)],2) < d
       cs(pm_index) = cs(pm_index) + 1; 
    end
    
    % With waypoint vector, "shrink" waypoint by amount determined by
    % uncertainties in upcoming hallway
    wypt_vec = [waypoint.x - x;waypoint.y - y];
    tmp = [x;y] + dist_frac*wypt_vec;
    waypoint.x = tmp(1);
    waypoint.y = tmp(2);
end