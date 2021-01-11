classdef jackal < UGV
   properties
       
   end
   methods
       function setParams(obj,map)
           % Max speed = 0.65 m/s
           clear obj.getcorner_MPC
           clear obj.getcorner_WP
           clear obj.get_wypt
           clear obj.motion_step
           obj.R = 0.075;
           obj.L = 0.42;
           obj.max_w = 26.667; % Max rotation for single wheel
           obj.max_omega = 1.5; % Max angular rotation for heading
           obj.max_v = 2.0;
           obj.size = 0.25; % m, max radius
           obj.dd = DifferentialDrive(obj.R,obj.L);
           obj.dt = 0.1;
           obj.t = 0.0;
           obj.x = map.pose_start.x;
           obj.y = map.pose_start.y;
           obj.theta = pi/2;
           obj.v = 0;
           obj.w = 0;
           obj.vx = 0;
           obj.vy = 0;
           obj.ax = 0;
           obj.ay = 0;
           obj.xc_mpc_r = map.corners_r(1,1);
           obj.yc_mpc_r = map.corners_r(1,2);
           obj.xc_mpc_l = -1*obj.large_num;
           obj.yc_mpc_l = obj.yc_mpc_r;
           obj.xc_wp = map.corners_r(1,1);
           obj.yc_wp = map.corners_r(1,2);
           obj.wypt.x = obj.x;
           obj.wypt.y = obj.y + obj.maxRad;
           obj.MPCinput.u = zeros(obj.N,obj.Nu);
           obj.trail.x = obj.x;
           obj.trail.y = obj.y;
           obj.proj_mot.x = obj.x*ones(1,obj.N);
           obj.proj_mot.y = obj.y*ones(1,obj.N);
           obj.maxRad = map.maxRad_suggest;
           obj.r = obj.maxRad;
           obj.getcorner_MPC(map);
           obj.last_sec = false;
       end
   end
end