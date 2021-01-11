close all;

v = VideoWriter("vid.avi","Uncompressed AVI");
open(v);

x = -0.5;
y = 0;
r = 3;
dt = 0.1;

global wall;
wall = true;

hl = 10;
hw = 10;
OOS = 20; % Out of screen coordinate
dl = 1; % door length
dw = 0.5; % door width
n = 10;

grid.n = n;
grid.probs = 0.5*ones(grid.n,1);
grid.occup = zeros(grid.n,1);
grid.xs = (hw/grid.n)*(0:n-1) + (-hw/2 + hw/grid.n/2);
grid.ys = (hl/2)*ones(1,n);
grid.p_appear = 0.5; % probability of appearing at doorway

% Plot things
figure(1);
hold on;


gray = 0.5*ones(1,3);
black = 0*ones(1,3);

patch([-OOS,-hw/2,-hw/2,-OOS],[0,0,OOS,OOS],gray); % left wall
patch([hw/2,OOS,OOS,hw/2],[0,0,OOS,OOS],gray); % right wall

if wall
   patch([0,hw/2,hw/2,0],[0,0,hl/2,hl/2],gray); 
end

patch([-hw/2-dw,-hw/2,-hw/2,-hw/2-dw],[hl/2-dl/2,hl/2-dl/2,hl/2+dl/2,hl/2+dl/2],black); % left door
patch([hw/2,hw/2+dw,hw/2+dw,hw/2],[hl/2-dl/2,hl/2-dl/2,hl/2+dl/2,hl/2+dl/2],black); % right door

plts_patches = {};
grid_ys = [-dl/2,-dl/2,dl/2,dl/2]+hl/2;

for i = 1:10
   plts_patches{i} = patch([i-1,i,i,i-1]-hw/2,grid_ys,grid.probs(i)*ones(1,3)); 
end

plt_robot = plot(x,y,"ro","MarkerSize",10,"MarkerFaceColor","r");
circ_pts = circle(x,y,r);
plt_FOV = plot(circ_pts.x,circ_pts.y,"b-");

axis equal;
xlim([-(0.6)*hw,0.6*hw]);
ylim([0,hl]);

% Simulation
flag = true;
while flag
   y = y + dt; 
   circ_pts = circle(x,y,r);
   
   % Update probs
   grid = update_probs(x,y,r,grid);
   
   % Update plots
   plt_robot.XData = x;
   plt_robot.YData = y;
   plt_FOV.XData = circ_pts.x;
   plt_FOV.YData = circ_pts.y;
   for i = 1:grid.n
      plt_patch = plts_patches{i};
      plt_patch.FaceColor = grid.probs(i)*ones(1,3);
   end
   
   if y > hl+r
      flag = false; 
   end
   
   frame = getframe(gcf);
   writeVideo(v,frame);
   
   pause(0.1);
end

close(v);
