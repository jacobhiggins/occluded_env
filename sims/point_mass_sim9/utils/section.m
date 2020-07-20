% Returns what section of the hallway the robot is in
function sec = section(x,y,hws,hls)
    offset = 0;
    if y < hls(1)-hws(2)-offset && x < hws(1)
       sec = 1;
    elseif x < hls(2) - hws(3) - offset
       sec = 2;
    else
       sec = 3;
    end
end