function [zs,stop,avg_P] = maximum(points,x_est,y_est,m_est,phi_est)
    stop = false;
%     P = exp( -( ( (points(:,2)-y_est) - repmat(m_est,size(points,1),1).*(points(:,1)-x_est) ) ).^2 );
%     zs = P./sum(P,2);
    
    P1 = exp( -( ( repmat(cos(phi_est),size(points,1),1).*(points(:,2)-y_est) - repmat(sin(phi_est),size(points,1),1).*(points(:,1)-x_est) ) ).^2/0.05 );
    zs = P1./sum(P1,2);
    avg_entropy = mean(-zs.*log(min(1,zs+0.0000001)));
    avg_P = mean(max(P1,[],2));
    if size(zs,2)==1 %|| (max(avg_entropy) < 0.05)
       stop = true; 
    end
end