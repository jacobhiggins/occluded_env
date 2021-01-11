function zs = init_zs(tot_points,num_lines)
    points_per_line = int16(tot_points/num_lines);
    zs = [];
    points_added = 0;
    for i = 1:num_lines-1
        M = zeros(points_per_line,num_lines);
        M(:,i)=1;
        zs = [zs;M];
        points_added = points_added + points_per_line;
    end
    M = zeros(tot_points-points_added,num_lines);
    M(:,end)=1;
    zs = [zs;M];
end