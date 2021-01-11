close all;

start_point1 = struct("x",1,"y",1);
end_point1 = struct("x",1,"y",8);
start_point2 = struct("x",2,"y",8);
end_point2 = struct("x",8,"y",8);
start_point3 = struct("x",8,"y",7);
end_point3 = struct("x",8,"y",3);
num_points = [20,20,20];
num_lines = 3;

points1 = [linspace(start_point1.x,end_point1.x,num_points(1))',...
    linspace(start_point1.y,end_point1.y,num_points(1))'];
points2 = [linspace(start_point2.x,end_point2.x,num_points(2))',...
    linspace(start_point2.y,end_point2.y,num_points(2))'];
points3 = [linspace(start_point3.x,end_point3.x,num_points(3))',...
    linspace(start_point3.y,end_point3.y,num_points(3))'];

points = [points1;points2;points3];
tot_points = size(points,1);
% zs = init_zs(tot_points,num_lines);
% zs = [];
% for i = 1:num_lines
%    M = zeros(num_points,num_lines);
%    M(:,i) = 1;
%    zs = [zs;M];  
% end

% zs = [ones(num_points,1);zeros(num_points,1)];
% zs = [zs ~zs];
% [x_est,y_est,m_est,phi_est] = estimate(points,zs);
% m_est = 2+m_est;
% x_est = 2+x_est;
% % y_est = -2+y_est;
% line_points = struct("x",[],"y",[]);
% line_points.x = [x_est - 2*cos(phi_est);x_est + 2*cos(phi_est)];
% line_points.y = [y_est - 2*sin(phi_est);y_est + 2*sin(phi_est)];
% line_points.y = m_est.*(line_points.x - repmat(x_est,2,1)) + y_est;

% figure(1);
% hold on;
% plt_points = scatter(points(:,1),points(:,2));
% plt_lines = [];
% for i = 1:num_lines
%     plt_line = plot(line_points.x(:,i),line_points.y(:,i));
%     plt_lines = [plt_lines plt_line];
% end
% xlim([0,10]);
% ylim([0,10]);
%%
close all;
figure(1);
for k = 1:num_lines
    
    zs = init_zs(tot_points,k);
    [x_est,y_est,m_est,phi_est] = estimate(points,zs);
    subplot(1,num_lines,k);
    hold on;
    plt_points = scatter(points(:,1),points(:,2),20,repmat([0,0.4470,0.7410],length(points),1),"filled");
    axis equal;
    plt_lines = [];
    line_points = struct("x",[],"y",[]);
    line_points.x = [x_est - 2*cos(phi_est);x_est + 2*cos(phi_est)];
    line_points.y = [y_est - 2*sin(phi_est);y_est + 2*sin(phi_est)];
    cmap = lines(k);
    for i = 1:k
        plt_line = plot(line_points.x(:,i),line_points.y(:,i),"Color",cmap(i,:));
        plt_lines = [plt_lines plt_line];
    end
    xlim([0,10]);
    ylim([0,10]);
    
    for i = 1:10
        [zs,stop,avg_P] = maximum(points,x_est,y_est,m_est,phi_est);
        [x_est,y_est,m_est,phi_est] = estimate(points,zs);
        line_points.x = [x_est - 50*cos(phi_est);x_est + 50*cos(phi_est)];
        line_points.y = [y_est - 50*sin(phi_est);y_est + 50*sin(phi_est)];
        for j = 1:k
            plt_line = plt_lines(j);
            plt_line.XData = line_points.x(:,j);
            plt_line.YData = line_points.y(:,j);
        end
        fprintf("line: %d, step: %d\n",k,i);
        if stop || i==10
            title(sprintf("lines: %d, average max prob: %f",k,avg_P));
            [vals,Is] = max(zs,[],2);
            cmap = lines(k);
            cmap_expanded = cmap(Is,:);
            plt_points.CData = cmap_expanded;
            break;
        end
        pause(0.5);
    end
end




% avg_point = mean(points1);
% 
% num = sum((points1(:,1)-avg_point(1)).*(points1(:,2)-avg_point(2)));
% dem = sum((points1(:,1)-avg_point(1)).^2);
% 
% m_est = num/dem;
% m_true = (end_point1.y - start_point1.y)/(end_point1.x - start_point1.x);

