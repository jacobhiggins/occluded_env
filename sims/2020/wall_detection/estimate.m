function [x_est,y_est,m_est,phi_est] = estimate(points,zs)
    % zs = latent values
    % rows = points
    % columns = latent values
    num_lines = size(zs,2);
    zks = sum(zs);
    x_est = (( zs'*points(:,1) )./zks')';
    y_est = (( zs'*points(:,2) )./zks')';
   
    var_x = sum(zs.*( (repmat(points(:,1),1,num_lines) - x_est).^2 ));
    
    m_est_num = sum(zs.*( (repmat(points(:,1),1,num_lines) - x_est).*(repmat(points(:,2),1,num_lines) - y_est) ));
    m_est = m_est_num./sum(zs.*( (repmat(points(:,1),1,num_lines) - x_est).^2 ));
    phi_est = atan(m_est_num./sum(zs.*( (repmat(points(:,1),1,num_lines) - x_est).^2 )));
    
    phi_est(var_x < 0.0000000001) = pi/2;
end