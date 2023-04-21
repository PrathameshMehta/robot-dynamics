function traj = make_trajectory(type, params)
% YOUR CODE HERE
if strcmp(type,'quintic')
    t = params.t;
    dt = params.dt;
    q = params.q; 
    v = params.v;
    a = params.a;
    
    P = [1 t(1) t(1)^2 t(1)^3 t(1)^4 t(1)^5;
         0 1 2*t(1) 3*t(1)^2 4*t(1)^3 5*t(1)^4;
         0 0 2 6*t(1) 12*t(1)^2 20*t(1)^3;
         1 t(2) t(2)^2 t(2)^3 t(2)^4 t(2)^5;
         0 1 2*t(2) 3*t(2)^2 4*t(2)^3 5*t(2)^4;
         0 0 2 6*t(2) 12*t(2)^2 20*t(2)^3];
    
    Q = [q(1);v(1);a(1);q(2);v(2);a(2)];
    A = linsolve(P,Q);
    
    n = (t(2)-t(1))/dt;
    traj.t = 0;
    traj.q = A(1);
    traj.v = A(2);
    traj.a = 2*A(3);
    for i = 1:n

        t = traj.t(end)+ dt;
        traj.t = [traj.t t];
        
        position = [1 t t^2 t^3 t^4 t^5]* A;
        traj.q = [traj.q position];

        velocity = [0 1 2*t 3*t^2 4*t^3 5*t^4]* A;
        traj.v = [traj.v velocity];

        accer = [0 0 2 6*t 12*t^2 20*t^3]* A;
        traj.a = [traj.a accer];

    end

elseif strcmp(type,'cubic')

    t = params.t;
    dt = params.dt;
    q = params.q; 
    v = params.v;
    
    P = [1 t(1) t(1)^2 t(1)^3;
         0 1 2*t(1) 3*t(1)^2;
         1 t(2) t(2)^2 t(2)^3;
         0 1 2*t(2) 3*t(2)^2];

    Q = [q(1);v(1);q(2);v(2)];
    A = linsolve(P,Q);
    
    n = (t(2)-t(1))/dt;
    traj.t = 0;
    traj.q = A(1);
    traj.v = A(2);
    traj.a = 2*A(3);
    
    for i = 1:n

        t = traj.t(end)+ dt;
        traj.t = [traj.t t];
        
        position = [1 t t^2 t^3]* A;
        traj.q = [traj.q position];

        velocity = [0 1 2*t 3*t^2]* A;
        traj.v = [traj.v velocity];
        
        accer = [0 0 2 6*t]* A;
        traj.a = [traj.a accer];

    end

 end
    

end
