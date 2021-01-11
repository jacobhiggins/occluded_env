classdef big_map < map
   properties
       portion1% = struct("corners_r",[0.0,0.0,0.0],"wall_mpc",[0.0],"Ms_mpc",{},"wypt_bases",[]);
       portion2% = struct("corners_r",[0.0,0.0,0.0],"wall_mpc",[0.0],"Ms_mpc",{},"wypt_bases",[]);
       portions;
       current_portion = 1;
       switch_portion = false;
   end
   methods
       function setParams(obj)
           % Left most wall
           wall1.x = [-1 0 0 -1];
           wall1.y = [0 0 38 38];
           obj.walls{1} = wall1;
           % Top most wall
           wall2.x = [-1 21 21 -1];
           wall2.y = [37 37 38 38];
           obj.walls{2} = wall2;
           % Right most wall
           wall3.x = [20 21 21 20];
           wall3.y = [0 0 32 32];
           obj.walls{3} = wall3;
           % Bottom middle wall
           wall4.x = [5 15 15 5];
           wall4.y = [0 0 10 10];
           obj.walls{4} = wall4;
           % Middle middle wall
           wall5.x = [-1 15 15 -1];
           wall5.y = [15 15 27 27];
           obj.walls{5} = wall5;
           % Upper middle wall
           wall6.x = [14 15 15 14];
           wall6.y = [30 30 38 38];
           obj.walls{6} = wall6;
           
           % Plot limits
           obj.plt_xlim = [-1 21];
           obj.plt_ylim = [0 38];
           
           % Start pose of robot
           obj.pose_start.x = 2.5;
           obj.pose_start.y = 0;
           
           % Set corners
           obj.set_corners();
           
           % Set waypoint bases
           obj.set_wypt_bases();
                      
           obj.set_patches2d();
           obj.set_modeltraj_points();
           
           % Store portions info in cell
           obj.portions = {obj.portion1,obj.portion2};
           
           % Maximum radius suggest
           obj.maxRad_suggest = 7;
       end
       
       function set_corners(obj)
           
           % Portion1
           corners_r = [5,10,1];
           Ms_mpc = {eye(2)};
           wall_mpc = -5;
           
           corners_r = [corners_r;[15,15,-1]];
           Ms_mpc = cat(2,Ms_mpc,[0 1;1 0]);
           wall_mpc = [wall_mpc -5];
           
           corners_r = [corners_r;[15,27,-1]];
           Ms_mpc = cat(2,Ms_mpc,[-1 0;0 1]);
           wall_mpc = [wall_mpc -5];
           
           corners_r = [corners_r;[14,30,1]];
           Ms_mpc = cat(2,Ms_mpc,[0 1;-1 0]);
           wall_mpc = [wall_mpc -3];
           
           corners_r = [corners_r;[14,37,1]];
           Ms_mpc = cat(2,Ms_mpc,[0 1;-1 0]);
           wall_mpc = [wall_mpc -10];
           
           obj.portion1.corners_r = corners_r;
           obj.portion1.Ms_mpc = Ms_mpc;
           obj.portion1.wall_mpc = wall_mpc;
           obj.portion1.corners_l = [0.0 0.0];
           
           % Portion2
           corners_r = [5 45 -1];
           Ms_mpc = {[0 1;1 0]};
           wall_mpc = -18;
           
           corners_r = [corners_r;14 30 -1];
           Ms_mpc = cat(2,Ms_mpc,[1 0;0 -1]);
           wall_mpc = [wall_mpc;-14];
           
           corners_r = [corners_r;15 30 -1];
           Ms_mpc = cat(2,Ms_mpc,[0 1;1 0]);
           wall_mpc = [wall_mpc;-3];
           
           corners_r = [corners_r;20 32 1];
           Ms_mpc = cat(2,Ms_mpc,eye(2));
           wall_mpc = [wall_mpc;-5];
           
           corners_r = [corners_r;30 32 1];
           Ms_mpc = cat(2,Ms_mpc,[0 -1;1 0]);
           wall_mpc = [wall_mpc;-5];
           
           obj.portion2.corners_r = corners_r;
           obj.portion2.Ms_mpc = Ms_mpc;
           obj.portion2.wall_mpc = wall_mpc;
           obj.portion2.corners_l = [0.0 0.0];
           
       end
       
       function set_wypt_bases(obj)
           % Portion 1
           wypt_bases = [obj.pose_start.x,obj.pose_start.y];
           wypt_bases = [wypt_bases;obj.pose_start.x,12.5];
           wypt_bases = [wypt_bases;17.5,wypt_bases(end,2)];
           wypt_bases = [wypt_bases;wypt_bases(end,1),28.5];
           wypt_bases = [wypt_bases;1.5,wypt_bases(end,2)];
           wypt_bases = [wypt_bases;wypt_bases(end,1),35.5];
           obj.portion1.wypt_bases = wypt_bases;
           
           %Portion2
           wypt_bases = [wypt_bases(end,:);12.5,35.5];
           wypt_bases = [wypt_bases;wypt_bases(end,1),28.5];
           wypt_bases = [wypt_bases;17.5,wypt_bases(end,2)];
           wypt_bases = [wypt_bases;wypt_bases(end,1),34.5];
           wypt_bases = [wypt_bases;31,wypt_bases(end,2)];
           obj.portion2.wypt_bases = wypt_bases;
       end
       
       function get_wypt(obj)
           
       end
       
       function value = isVisible(obj,corner,robot)
           value = (norm([robot.x-corner.x,robot.y-corner.y])<=robot.r);
       end
       
       function check_flag(obj,p)
           if p.x > 21
              obj.end_flag = false; 
           end
       end
       
       function update_traj(obj)
           figure(1);
           obj.plt_traj.XData = obj.portion2.wypt_bases(:,1);
           obj.plt_traj.YData = obj.portion2.wypt_bases(:,2);
       end
       
       function set_patches2d(obj)
           obj.patches.dim.x = 4;
           obj.patches.dim.y = 25;
           obj.patches.start.x = 15;
           obj.patches.start.y = 0;
           obj.patches.end.x = 20;
           obj.patches.end.y = 37;
           obj.patches.probs = 1.0*ones(obj.patches.dim.x,obj.patches.dim.y);
           obj.patches.width = (obj.patches.end.x - obj.patches.start.x)/obj.patches.dim.x;
           obj.patches.height = (obj.patches.end.y - obj.patches.start.y)/obj.patches.dim.y;
           obj.patches.centers.x = (obj.patches.width)*(0:obj.patches.dim.x-1) + obj.patches.width/2 + obj.patches.start.x;
           obj.patches.centers.y = (obj.patches.height)*(0:obj.patches.dim.y-1) + obj.patches.height/2 + obj.patches.start.y;
           [X,Y] = meshgrid(obj.patches.centers.x,obj.patches.centers.y);
           obj.patches.centers.meshX = X;
           obj.patches.centers.meshY = Y;
       end
        
       function update_probs2d(obj,p)
            persistent last_sample_time;
            if isempty(last_sample_time)
               last_sample_time = 0; 
            elseif (p.t - last_sample_time) < 1/p.MPC_Hz
               return; 
            end
            last_sample_time = p.t;
            p_z1_m1 = 0.9;
            p_z1_m0 = 1 - p_z1_m1;
            p_z0_m0 = 0.9;
            p_z0_m1 = 1 - p_z0_m0;
            p_move = 0.08;
            p_stay = 1 - p_move;
            for i = 1:obj.patches.dim.x
               for j = 1:obj.patches.dim.y
                   point_x = obj.patches.centers.meshX(j,i);
                   point_y = obj.patches.centers.meshY(j,i);
                   [x_point_prime,y_point_prime] = c2u(point_x,point_y,p.x,p.y,[0 1;1 0]);
                   theta_point = atan2(y_point_prime,x_point_prime);
                   [x_robot_prime,y_robot_prime] = c2u(p.xc_mpc_r,p.yc_mpc_r,p.x,p.y,[0 1;1 0]);
                   theta_vis = atan2(y_robot_prime,x_robot_prime);
                   d = norm([point_x-p.x,point_y-p.y]);
                   
                   vis = true;
                   if obj.current_portion==1 && p.current_sec==2
                      [x_point_prime2,y_point_prime2] = c2u(15,10,p.x,p.y,[0 1;1 0]);
                      theta_vis2 = atan2(y_point_prime2,x_point_prime2);
                      if theta_point > theta_vis2 || theta_point < theta_vis
                         vis = false; 
                      end
                   end
                   
                   if (obj.current_portion==1 && p.current_sec>=6) || (obj.current_portion==2 && p.current_sec<=3)
                      [x_point_prime2,y_point_prime2] = c2u(15,30,p.x,p.y,[0 1;1 0]);
                      theta_vis2 = atan2(y_point_prime2,x_point_prime2);
                      [x_point_prime3,y_point_prime3] = c2u(15,27,p.x,p.y,[0 1;1 0]);
                      theta_vis3 = atan2(y_point_prime3,x_point_prime3);
                      if theta_point > theta_vis3 || theta_point < theta_vis2
                         vis = false; 
                      end
                   end
                   
                   if (d < p.maxRad && vis) || (d < p.maxRad && ( (p.current_sec==3 && obj.current_portion==1) || (p.current_sec==4 && obj.current_portion==2)))
                       if rand < p_z0_m0
                           obj.patches.probs(i,j) = p_z0_m0*obj.patches.probs(i,j)/(p_z0_m0*obj.patches.probs(i,j) + p_z0_m1*(1-obj.patches.probs(i,j)));
                       else
                           obj.patches.probs(i,j) = p_z1_m0*obj.patches.probs(i,j)/(p_z1_m0*obj.patches.probs(i,j) + p_z1_m1*(1-obj.patches.probs(i,j)));
                       end
                   end
               end
            end
            
            % Convolution with motion
            if true%abs(obj.hls(1)-obj.hws(2)) > p.y
               for t = p.dt:p.dt:abs(p.x - 17.5)/p.max_v
                   for i = 1:obj.patches.dim.x
                      for j = 1:obj.patches.dim.y-1
                          obj.patches.probs(i,j) = p_move*obj.patches.probs(i,j+1) + p_stay*obj.patches.probs(i,j);
                      end
                   end
               end
            end
            
       end
       
       function set_modeltraj_points(obj)
            num_points = 100; % 100 points between each wypt base
            obj.portion1.model_traj = struct("points",[]);
            for i = 1:size(obj.portion1.wypt_bases,1)-1
                wypta = obj.portion1.wypt_bases(i,:);
                wyptb = obj.portion1.wypt_bases(i+1,:);
                vec = (wyptb - wypta);
                vec = vec/num_points;
                points = (1:num_points)'*vec + wypta;
                obj.portion1.model_traj.points = [obj.portion1.model_traj.points;points];
            end
            obj.portion2.model_traj = struct("points",[]);
            for i = 1:size(obj.portion2.wypt_bases,1)-1
                wypta = obj.portion2.wypt_bases(i,:);
                wyptb = obj.portion2.wypt_bases(i+1,:);
                vec = (wyptb - wypta);
                vec = vec/num_points;
                points = (1:num_points)'*vec + wypta;
                obj.portion2.model_traj.points = [obj.portion2.model_traj.points;points];
            end
        end
       
   end
end