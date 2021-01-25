function [points,loc,heading,index] = sqr_points(robot)

    is = logical(robot.trail.x < 3.9) .* logical(robot.trail.x > 3.8) .* ...
        logical(robot.trail.y < 13.7) .* logical(robot.trail.y > 13.6);
    is = find(logical(is));
    index = is(1);

    x = robot.trail.x(index);
    y = robot.trail.y(index);
    heading = [robot.vs.vxs(index);robot.vs.vys(index)];
    heading = heading/norm(heading);
    angle = atan2(heading(2),heading(1));
    
    
    
end