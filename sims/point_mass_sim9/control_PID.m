function [ax,ay,ex,ey] = control_PID(xr,yr,xg,yg,int_ex,int_ey,dt,ex_prev,ey_prev)
    kp = 2;
    ki = 5;
    kd = 9;
    
    ex = xg - xr;
    ey = yg - yr;
    
    int_ex = int_ex + ex*dt;
    int_ey = int_ey + ey*dt;
    
    d_ex = (ex - ex_prev)/dt;
    d_ey = (ey - ey_prev)/dt;
    
    ax = kp*ex + ki*int_ex + kd*d_ex;
    ay = kp*ey + ki*int_ey + kd*d_ey;
end