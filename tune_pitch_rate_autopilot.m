function res = tune_pitch_rate_autopilot(params, K, sp)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    tspan = [0, 10];
    t = tspan(1);
    dt = 0.01;
    
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.cost = 0;
    
    %% Pitch rate autopilot instances
    int_accum = 0;
    int_accum_2 = 0;
    
    while t < tspan(2)
        %% 4-loop PID altitude autopilot
        euler = euler_angles(y(10:13));
        e1 = sp - euler(2);
        u1 = K(1) * e1;
        
        
        e2 = u1 - y(8);
        u2 = K(2) * e2 + K(3) * int_accum;
        int_accum = int_accum + e2 * dt;
        
        if t > 2.0
            u1a = K(4) * e1;
            e2a = u1a - y(8);
            u2a = K(5) * e2a + K(6) * int_accum_2;
            int_accum_2 = int_accum_2 + e2a * dt;
        else
            u2a = 0.0;
        end
        
        params.deltas_cmd(1) = max(min(u2a, deg2rad(15)), deg2rad(-15));
        params.vanes_cmd(1) = max(min(u2, deg2rad(15)), deg2rad(-15));
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control, stop the simulation and penalize cost value
        if norm(y(7:9)) > 200.0
             res.cost = res.cost + 100;
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        
        res.cost = res.cost + 100*e1^2*dt;
    end
    
    %% Calculate rise time, setting time and overshoot (we want to minimize the sum of them)
    euler = euler_angles(res.y(:,10:13));
    info = stepinfo(euler(:,2), res.t, sp);
    if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
        res.cost = res.cost + 25;
    else
        res.cost = res.cost + info.RiseTime + info.SettlingTime + abs(info.Overshoot - 4.598791);
    end
end