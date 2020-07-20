function unknowns = find_unknowns(x,y,ranges,max_range,scan_angles,map)
    del_r = 5;
%     unknown_xs = zeros(scan_angles,int16(max_range/del_r));
%     unknown_ys = zeros(scan_angles,int16(max_range/del_r));
    del_ranges = ranges - max_range*ones(length(ranges),1);
    is = find(del_ranges<0); 
    ranges = ranges(is); % Find all range values less than max
    scan_angles = scan_angles(is); % Find associated scan angles
%     xs = x*ones(1,length(ranges)) + ranges'.*cos(scan_angles-pi/2);
%     ys = y*ones(1,length(ranges)) + ranges'.*sin(scan_angles-pi/2);
    xs = [];
    ys = [];
    for i = 1:length(scan_angles)
        scan_angle = scan_angles(i);
        range = ranges(i);
        while range < max_range
            xs = [xs;x + range*cos(scan_angle-pi/2)];
            ys = [ys,y + range*sin(scan_angle-pi/2)];
            range = range + del_r;
        end
    end
    unknowns.xs = xs;
    unknowns.ys = ys;
end