function cost = MPCcost_wper(x0,u)
    global ref;
    global NP; % Prediction horizon
    global NC; % Control Horizon
    global A;
    global B;
    global Q;
    global R;
    global dt;
    global w_perc;
    persistent np;
    persistent ref_mat;
    persistent Q_mat;
    persistent R_mat;
    persistent A_mat;
    persistent B_mat;
    
    if isempty(np)
       np = size(u,2);
       ref_mat = repmat(ref,NP,np);
       A_mat = [];
       prev_term = eye(2);
       for i = 1:NP
          A_mat = [A_mat;
              A^i];
       end
       B_mat = [];
       term = [];
       for i = 1:NP
           term = [A^(i-1)*B term];
           B_mat = [B_mat;
               term  repmat(zeros(size(B)),1,NP-i)];
       end
       Q_mat = repmat(Q,NP,np);
       R_mat = repmat(R,NP,np);
    end
    
    % Array of particles is of size num_ctrl x np
    % for repmat([],x,y), x is usually contorl horizon, y is np
    
    % First, calculate the 
    x0_mat = repmat(x0,1,np);
    x0_prop = A_mat*x0_mat;
    u_mat = [u;repmat(u(end-1:end,:),NP-NC,1)];
    u_prop = B_mat*u_mat*dt;
    x_prop = x0_prop + u_prop;
    e_state = x_prop - ref_mat;
    
    perc_val = perc(x_prop',NP,np);

    cost = sum(e_state.*Q_mat.*e_state,1) + sum(u_prop.*R_mat.*u_prop,1) + w_perc*perc_val;
    
end