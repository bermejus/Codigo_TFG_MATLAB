function res = simulation(params, K, sp, tf)
    tspan = [0.0, tf];
    dt = 0.01;
    t = tspan(1);
    
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18.0), [0; 1; 0]);
    
    res.dy = dynamics(t, y, params)';
    
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.data = zeros(1,2);
    res.cost = 0.0;
    
    Kp_t = K(1); Ki_t = K(2); Kd_t = K(3);
    Kp_a = K(4); Ki_a = K(5); Kd_a = K(6);
    
    prev_error = (sp + y(3))/sp;
    int_accum = 0.0;
    
    while t < tspan(2)
        % PID Autopilot
        error = (sp + y(3))/sp;
        int_accum = int_accum + (error + prev_error) * dt;
        
        if t < params.tb
            ut_cmd = Kp_t * error + Ki_t * int_accum + Kd_t * (error - prev_error) / dt;
            params.vanes_cmd(1) = max(min(ut_cmd, deg2rad(15.0)), -deg2rad(15.0));
        else
            params.vanes_cmd = [0, 0, 0];
        end
        
        ua_cmd = Kp_a * error + Ki_a * int_accum + Kd_a * (error - prev_error) / dt;
        params.deltas_cmd(1) = max(min(ua_cmd, deg2rad(15.0)), -deg2rad(15.0));
        prev_error = error;
        
        % Run for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        if y(3) >= 0.0 || norm(y(7:9)) > 1000.0
             res.cost = res.cost + 5;
             break;
        end
        
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
        res.data = [res.data; [params.vanes_cmd(1) params.deltas_cmd(1)]];
        
        res.cost = res.cost + error^2*dt;
    end
end