function [v,v_max,delta_x] = safety_vel(x,y,xc,yc,xm2,ym2,theta2,hw2)
    delta_x = 50;
    a_max = 1;
    v_max = sqrt(2*delta_x*a_max);
    theta = atan2(yc-y,xc-x);
    if theta ~= theta2
        A = [cos(theta),-cos(theta2);...
            sin(theta),-sin(theta2)];
        b = [xm2-x;ym2-y];
        vec = A\b;
        delta_x = vec(1);
    end
    v = min(sqrt(2*a_max*min(delta_x,ym2+hw2-y)),v_max);
%     v = min(sqrt(2*a_max*delta_x),v_max);
end