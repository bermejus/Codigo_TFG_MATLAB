function res = tune_altitude_autopilot(params, K, sp, print_snd_order_info)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    if ~exist("print_snd_order_info", "var")
        print_snd_order_info = false;
    end
    
    tspan = [0, 20];
    t = tspan(1);
    dt = 0.01;
    
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.cost = 0;
    
    %% Pitch rate autopilot instances
    pid = AltitudeAutopilot(K, deg2rad(20) * [-1,-1], deg2rad(20) * [1,1]);
    
    while t < tspan(2)
        %% PID altitude autopilot
        u_cmd = pid.output(1.0, y(3)/sp, dt);
        if t < 2.0
            params.vanes_cmd(1) = u_cmd(1);
        else
            params.vanes_cmd(1) = 0;
        end
        params.deltas_cmd(1) = u_cmd(2);
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control, stop the simulation and penalize cost value
        if y(3) > 0 || norm(y(7:9)) > 200.0
             res.cost = res.cost + 30;
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        
        res.cost = res.cost + 10*pid.error^2*dt;
    end
    
    %% Calculate rise time, setting time and overshoot (we want to minimize the sum of them)
    ts = res.t;
    ys = -res.y(:,3);
    
    info = stepinfo(ys, ts, -sp);
    res.RiseTime = info.RiseTime;
    res.SettlingTime = info.SettlingTime;
    res.Overshoot = info.Overshoot;
    res.damp = fzero(@(x) (exp(-pi*x/sqrt(1-x^2)) - info.Overshoot/100), 0.5);
    
    if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
        res.cost = res.cost + 30;
    else
        res.cost = res.cost + info.RiseTime + info.SettlingTime + abs(info.Overshoot - 4.598791);
    end
    
    % Print relevant information
    if print_snd_order_info
        fprintf("Rise time: %g s\n", res.RiseTime);
        fprintf("Settling time: %g s\n", res.SettlingTime);
        fprintf("Overshoot: %g %c\n", res.Overshoot, "%");
        fprintf("Damping: %g\n", res.damp);
    end
end