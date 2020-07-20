function r=rad_intersect(theta1,theta2,x10,x20,y10,y20)
    if theta1~=theta2
        A = [cos(theta1),-cos(theta2);...
            sin(theta1),-sin(theta2)];
        b = [x20-x10;y20-y10];
        r = A\b;
    end
end