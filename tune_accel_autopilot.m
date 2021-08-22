function res = tune_accel_autopilot(params, Kh, K, sp)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    tspan = [0, 7];
    t = tspan(1);
    dt = 0.01;
    
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.dy = dynamics(t, y, params)';
    res.cost = 0;
    
    %% Height controller
    int_accum = 0;
    prev_error = 0;
    
    %% Acceleration LQR controller
    controller = Az_lqr(K, deg2rad(15) * [-1 -1], deg2rad(15) * [1 1]);
    
    while t < tspan(2)
        %% Autopilot loop
        if t < 5.0
            % Height control
            sp_h = -160;
            error = (sp_h - y(3)) / sp_h;
            
            if t < 2.0
                u_t = Kh(1) * error;
            else
                u_t = 0;
            end
            
            int_accum = int_accum + 0.5 * (error + prev_error) * dt;
            u_a = Kh(2) * error + Kh(3) * int_accum + Kh(4) * (error - prev_error) / dt;
            prev_error = error;
            
            params.vanes_cmd(1) = max(min(u_t, deg2rad(15)), deg2rad(-15));
            params.deltas_cmd(1) = max(min(u_a, deg2rad(15)), deg2rad(-15));
        else
            % Acceleration control
            u = controller.output(t, y, params, sp, dt);
            res.cost = res.cost + (controller.error^2)*dt;
            params.deltas_cmd(1) = u(1);
            
            if t < params.tb
                params.vanes_cmd(1) = u(2);
            else
                params.vanes_cmd(1) = 0;
            end
        end
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control, stop the simulation and penalize cost value
        if norm(y(7:9)) > 200.0
             res.cost = res.cost + 25000;
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
    end
    
    %% Calculate rise time, setting time and overshoot (we want to minimize the sum of them)
%     info = stepinfo(-res.y(:,3), res.t, sp);
%     if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
%         res.cost = res.cost + 100;
%     else
%         res.cost = res.cost + info.RiseTime + info.SettlingTime + info.Overshoot;
%     end
end