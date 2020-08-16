function vis = visFunction(X,Y)
    global hw1;
    global hw2;
    global radMax;
    
    vis.Data = [];
    vis.xnn = zeros(size(X,1)*size(X,2),2);
    vis.ynn = zeros(size(X,1)*size(X,2),1);
    vis.polygon = cell(size(X'));
    w = waitbar(0, "Creating Data Points");
    count = 0;
    % For each point in the first hallway
    for i = 1:size(X,1)
        for j = 1:size(X,2)
            x = X(i,j);
            y = Y(i,j);
            % Get upper wall interception point
            try
                [xus,yus] = linecirc(0,hw2,x,y,radMax);
                xu = xus(1);
                yu = yus(1);
                if xus(2) > xu
                    xu = xus(2);
                    yu = yus(2);
                end
            catch
                
            end
            % Get lower wall interception point
            try
                [xls,yls] = linecirc(0,0,x,y,radMax);
                xl = xls(1);
                yl = yls(1);
                if xls(2) > xl
                    xl = xls(2);
                    yl = yls(2);
                end
            catch
                
            end
            % Get mid point
            phi = atan2(-x,-y);
            R = (hw2 - y)/cos(phi);
            r = min(R,radMax);
            xm = x + r*sin(phi);
            ym = y + r*cos(phi);
            points = [];
            if xm < xu
                points = [0,0;
                xm,ym;
                xu,yu;
                xl,yl];
            else
                points = [0,0;
                xm,ym;
                xl,yl];
            end
            
            vis.polygon{i,j} = points;

            count = count + 1;
            vis.xnn(count,1) = x;
            vis.xnn(count,2) = y;
            if norm([x,y],2)>radMax
                vis.ynn(count) = 0;
                vis.Data{i,j} = 0;
            else
                area = polyarea(points(:,1),points(:,2));
                vis.ynn(count) = area;
                vis.Data{i,j} = area;
            end
            waitbar(count/(size(X,1)*size(X,2)),w,"Creating Data");
        end
    end
    delete(w);
end