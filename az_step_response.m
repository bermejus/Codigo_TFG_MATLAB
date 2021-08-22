function res = az_step_response(params, Kh, Kaz, sp, t_trigger, t_end)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    if ~exist("t_end", "var")
        t_end = 20.0;
    end
    
    tspan = [0, t_end];
    t = tspan(1);
    dt = 0.01;
    
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.dy = dynamics(t, y, params)';
    res.cost = 0;
    
    %% Altitude controller
    altitude_pid = AltitudeAutopilot(Kh, deg2rad(20) * [-1,-1], deg2rad(20) * [1,1]);
    
    %% Acceleration controller
    az_tvc_pid = AccelAutopilot(Kaz(1:3), -deg2rad(20), deg2rad(20));
    az_ae_pid = AccelAutopilot(Kaz(4:6), -deg2rad(20), deg2rad(20));
    guidance_activated = false;
    
    while t < tspan(2)
        %% Guidance loop
        if t >= t_trigger
            guidance_activated = true;
        end
        
        %% Autopilot loop
        if ~guidance_activated
            % Altitude control loop
            sp_h = -150;
            u_cmd = altitude_pid.output(1.0, y(3)/sp_h, dt);
            if t < 2.0
                params.vanes_cmd(1) = u_cmd(1);
            else
                params.vanes_cmd(1) = 0;
            end
            params.deltas_cmd(1) = u_cmd(2);
        else
            % Acceleration control loop
            dy = dynamics(t, y, params);
            fz_fb = dy(6) - y(4)*y(8);
            
            if t < params.tb
                u_tvc_cmd = az_tvc_pid.output(sp, fz_fb, y(8), dt);
            else
                u_tvc_cmd = 0;
            end
            u_ae_cmd = az_ae_pid.output(sp, fz_fb, y(8), dt);
            
            params.vanes_cmd(1) = u_tvc_cmd;
            params.deltas_cmd(1) = u_ae_cmd;
        end
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control
        if norm(y(7:9)) > 100.0
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
    end
    
    %% Calculate rise time, settling time and overshoot
    indices = res.t >= t_trigger;
    ts = res.t(indices);
    ys = res.dy(indices,6) - res.y(indices,4).*res.y(indices,8);
    info = stepinfo(ys, ts, sp);
    
    res.RiseTime = info.RiseTime;
    res.SettlingTime = (info.SettlingTime - t_trigger);
    res.Overshoot = info.Overshoot;
    res.damp = fzero(@(x) (exp(-pi*x/sqrt(1-x^2)) - info.Overshoot/100), 0.5);
    
    [~, locs] = findpeaks(ys, ts);
    res.natural_freq = 1/((locs(2) - locs(1)) * sqrt(1-res.damp^2));
    
    % Print relevant information
    fprintf("Rise time: %g s\n", res.RiseTime);
    fprintf("Settling time: %g s\n", res.SettlingTime);
    fprintf("Overshoot: %g %c\n", res.Overshoot, "%");
    fprintf("Damping: %g\n", res.damp);
    fprintf("Natural frequency: %g Hz\n", res.natural_freq);
end