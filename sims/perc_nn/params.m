function params()
    global NP;
    global NC;
    global ref;
    global feas;
    global A;
    global B;
    global Q;
    global R;
    global w_perc;
    global dt;
    global radMax;
    
    NP = 20;
    NC = 3;
    ref = [0;15];
    feas = [-5,5;
        -5,5];
    A = eye(2);
    B = eye(2);
    Q = [1;1];
    R = [0;0];
    dt = 1;
    radMax = 31;
    w_perc = 50;
end