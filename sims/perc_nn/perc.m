% X is np x 2N
% 2 = (x,y)
% np = number of particles in PSO
% N = control horizon
function perc_val = perc(X,N,np)
    global radMax;
    perc_val = zeros(1,np);
    split = mat2cell(X,np,2*ones(1,N));
    for i = 1:N
       x = split{i};
       tmp = ones(1,np);
       tmp(x(:,2)>-0.5)=0; % If past corner, unknown is zero
       tmp(vecnorm(x',2)'>radMax)=0; % If too far away, unknown is zero
       perc_val = perc_val + tmp.*(visNN2(x)').^2;
    end
    
end