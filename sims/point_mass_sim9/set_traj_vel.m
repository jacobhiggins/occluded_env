function v = set_traj_vel(x)
    v_fast = 10;
    v_safe = 6;
    v_cautious = 3;
    v_stop = 0;
    
    if x==1
        v = v_fast;
    elseif x==2
        v = v_safe;
    elseif x==3
        v = v_cautious;
    elseif x==4
        v = v_stop;
    end
end