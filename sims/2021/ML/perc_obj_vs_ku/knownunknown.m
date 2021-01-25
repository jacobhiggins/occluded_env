function ku = knownunknown(p)
            ku.area = 0;
            ku.poly.x = [-1000,-999,-999,-1000];
            ku.poly.y = [-1000,-1000,-999,-999];
            xc = 0;
            yc = 0;
           hw2 = abs(p.next_owall);
           
           if norm([p.x,p.y],2)>p.maxRad || p.y > 0.0
              return 
           end
           % Get upper wall interception point
           try
                [xus,yus] = linecirc(0,hw2,p.x,p.y,p.maxRad);
                xu = xus(1);
                yu = yus(1);
                if xus(2) > xu
                    xu = xus(2);
                    yu = yus(2);
                end
            catch
                
           end 
            
           % Get lower wall interception point
            try
                [xls,yls] = linecirc(0,0,p.x,p.y,p.maxRad);
                xl = xls(1);
                yl = yls(1);
                if xls(2) > xl
                    xl = xls(2);
                    yl = yls(2);
                end
            catch
                return
            end
            
            % Get mid point
            phi = atan2(-p.x,-p.y);
            R = (hw2 - p.y)/cos(phi);
            r = min(R,p.maxRad);
            xm = p.x + r*sin(phi);
            ym = p.y + r*cos(phi);
            m = [xm;ym] + [xc;yc];
            try
                u = [xu;yu] + [xc;yc];
            end
            try
                l = [xl;yl] + [xc;yc];
            end
            phi_1 = atan2(yl-p.y,xl-p.x);
            if xm < xu && ~isempty(xu)
                % Extra points for good curvature
                phi_2 = atan2(yu-p.y,xu-p.x);
%                 del_phi = phi_2 - phi_1;
                extra = ([p.x;p.y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
                
                ku.poly.x = [xc;m(1);u(1);extra(1,:)';l(1)];
                ku.poly.y = [yc;m(2);u(2);extra(2,:)';l(2)];
                
            elseif ~isempty(xl)
                phi_2 = atan2(ym-p.y,xm-p.x);
                extra = ([p.x;p.y] + p.maxRad*[cos(phi_2:-0.157:phi_1);sin(phi_2:-0.157:phi_1)]) + [xc;yc];
                
                
                ku.poly.x = [xc;m(1);extra(1,:)';l(1)];
                ku.poly.y = [yc;m(2);extra(2,:)';l(2)];
            end
            ku.area = polyarea(ku.poly.x,ku.poly.y);
           
end