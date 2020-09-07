classdef person_prob < handle
   properties
       mean = struct("x",0.0,"y",0.0,"vx",0.0,"vy",0.0);
       var = struct("x",0.0,"y",0.0,"vx",0.0,"vy",0.0);
       patches = struct("num",101,"w",0.2,"probs",[],"index_range",[],"centers",struct("local",[],"global",[])); % keep patches.num odd so have centered patch
       plts;
       rad_probs = struct("r",[],"prob",[]);
       mask = [];
   end
   methods
       function setParams(obj,m)
           obj.mean.x = m.hws(1) + 12;%7.3;
           obj.mean.y = m.hls(1) - 0.9*m.hws(2);
           obj.mean.vx = -1.0;
           obj.mean.vy = 0.0;
           obj.var.x = 1.0;
           obj.var.vx = 2;
           obj.patches.index_range = -floor(obj.patches.num/2):floor(obj.patches.num/2);
           obj.patches.centers.local = obj.patches.index_range*obj.patches.w; % centers local to person coordinate frame (centered on mean of distribution)
           obj.patches.centers.global = obj.patches.centers.local + obj.mean.x;
           obj.patches.probs = obj.patches.w*normpdf(obj.patches.centers.local,0,obj.var.x);
           obj.mask = zeros(size(obj.patches.probs));
       end
       function init_plot(obj,m)
           hold on;
           for i = 1:obj.patches.num
              x1 = obj.patches.centers.global(i) - obj.patches.w/2;
              x2 = obj.patches.centers.global(i) + obj.patches.w/2;
              y1 = m.hls(1) - m.hws(2);
              y2 = m.hls(1) - 0.5*m.hws(2);
              color = hsv2rgb([0.6 obj.patches.probs(i)/max(obj.patches.probs) 0.99]);
              plt = patch([x1 x2 x2 x1],[y1 y1 y2 y2],color,"EdgeColor","none","HandleVisibility","off");
              obj.plts = [obj.plts plt];
           end
       end
       function move(obj,dt)
          obj.mean.x = obj.mean.x + obj.mean.vx*dt;
          obj.mean.y = obj.mean.y + obj.mean.vy*dt;
          obj.var.x = obj.var.x + dt^2*obj.var.vx;
          obj.var.y = obj.var.y + dt^2*obj.var.vy;
          obj.patches.probs = obj.patches.w*normpdf(obj.patches.centers.local,0,obj.var.x);
          obj.patches.centers.global = obj.patches.centers.local + obj.mean.x;
       end
       function update_plot(obj)
          for i = 1:obj.patches.num
              x1 = obj.patches.centers.global(i) - obj.patches.w/2;
              x2 = obj.patches.centers.global(i) + obj.patches.w/2;
              color = hsv2rgb([0.6 (1 - obj.mask(i))*(obj.patches.probs(i)/max(obj.patches.probs)) 0.99]);
              plt = obj.plts(i);
              plt.XData = [x1 x2 x2 x1];
              plt.FaceColor = color;
          end
       end
       function update_radprobs(obj,r)
           xr = r.x;
           yr = r.y;
           xc = r.xc_mpc_r;
           yc = r.yc_mpc_r;
           theta_r = atan2(yc-yr,xc-xr);
           patches_ys = obj.mean.y*ones(1,obj.patches.num);
           patches_xs = obj.patches.centers.global;
           theta_probs = atan2(patches_ys - yc,patches_xs - xc);
           dists = sqrt((patches_ys - yr).^2 + (patches_xs - xr).^2);
           mask_new = (theta_probs > theta_r).*(dists < r.maxRad);
           mask_diff = mask_new - obj.mask;
           if norm(mask_diff,1) > 0
               mask_diff = logical(mask_diff);
               patches_probs = obj.patches.probs(mask_diff);
               dists = dists(mask_diff);
               obj.rad_probs.r = [obj.rad_probs.r dists];
               obj.rad_probs.prob = [obj.rad_probs.prob patches_probs];
           end
           obj.mask = min(mask_new+obj.mask,1);
       end
   end
end