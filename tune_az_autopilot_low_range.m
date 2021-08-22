function res = tune_az_autopilot_low_range(params, Kh, Kaz_tvc, Kaz_ae, sp)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    tspan = [0, 3.0];
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
    az_tvc_pid = AccelAutopilot(Kaz_tvc, -deg2rad(20), deg2rad(20));
    az_ae_pid = AccelAutopilot(Kaz_ae, -deg2rad(20), deg2rad(20));
    guidance_activated = false;
    
    while t < tspan(2)
        %% Guidance loop
        if y(3) < -10
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
                %res.cost = res.cost + 0.01*az_tvc_pid.error^2*dt;
            else
                u_tvc_cmd = 0;
            end
            u_ae_cmd = az_ae_pid.output(sp, fz_fb, y(8), dt);
            
            params.vanes_cmd(1) = u_tvc_cmd;
            params.deltas_cmd(1) = u_ae_cmd;
            
            res.cost = res.cost + 0.01*az_ae_pid.error^2*dt;
        end
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control, stop the simulation and penalize cost value
        if norm(y(7:9)) > 100.0
             res.cost = res.cost + 1000;
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
    end
    
    %% Calculate rise time, setting time and overshoot (we want to minimize the sum of them)
%     indices = res.t >= 1.07;
%     info = stepinfo(res.dy(indices,6) - res.y(indices,4).*res.y(indices,8), res.t(indices), sp);
%     if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
%         res.cost = res.cost + 50;
%     else
%         res.cost = res.cost + info.RiseTime + (info.SettlingTime - 1.07) + abs(info.Overshoot - 4.598791);
%     end
end