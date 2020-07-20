% This function explores different velocity profiles for the pursuit
% based on distance to corner and angle wrt corner
% vel5 function is the best so far
function v_set = set_traj_vel2(d,hw,phi_norm,phi)
    v_f = 10;
    v_s = 2;
    v_set = 10;
    D = 35;
%     disp(phi_norm);
%     if D<hw
%        v_set = v_s + max(phi_norm*v_f-v_s,0)*D/hw;
%     end

    delta = 0.16;
    A = [1 0 0 0;1 D D^2 D^3;0 1 0 0;0 1 2*D 3*D^2];
    b = [v_s;v_f;0;0];
    c = A\b;
    vel3 = @(d) ([ones(size(d)),d,d.^2,d.^3]*c).*(d<=35) + v_f*(d>35);
    phi3 = @(theta) (theta - pi/4)^2/((theta-pi/4)^2 + (15*pi/180)^2);
%     if true
%        v_set =  max(vel3(d)*phi3(phi),0.5);
%     end
%     if d<hw

    f= 0.9;
    A = [1 0 0 0 0;...
        1 D D^2 D^3 D^4;...
        0 1 0 0 0;...
        0 1 2*D 3*D^2 4*D^3;...
        0 0 2 6*f*D 12*(f*D)^2];
    b = [v_s;v_f;0;0;0];
    c = A\b;
    
    vel4 = @(d) ([1,d,d^2,d^3,d^4]*c).*(d<=35) + v_f*(d>35);
    v_set = vel4(d);

    delta = 5;
    phi_norm = 1 - phi_norm;
    if phi_norm < 0.5
        phi_norm = 2*phi_norm;
    else
        phi_norm = 1;
    end
    f= 5 + (D-2*delta)*phi_norm;
    A = [1 -delta delta^2 -delta^3;...
        1 delta delta^2 delta^3;...
        0 1 -2*delta 3*delta^2;...
        0 1 2*delta 3*delta^2];
    b = [v_s;v_f;0;0];
    c = A\b;
    vel5 = @(d) v_s*(d<f-delta) + ([1,(d-f),(d-f)^2,(d-f)^3]*c).*(f-delta<=d && d<=f+delta) + v_f*(d>f+delta);
    v_set = vel5(d);


%     if hw/4<D && D<3*hw/4
%        v_set = v_s + (v_f-v_s)*(D - hw/4)/(hw/2); 
%     elseif D<hw/4
%        v_set = v_s;
%     end
end