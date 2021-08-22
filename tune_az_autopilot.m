function res = tune_az_autopilot(params, Kh, Kaz_tvc, Kaz_ae, x_0, v_0)
    %% Initial conditions
    y = zeros(17,1);
    y(1:3) = [0; 0; -1];
    y(4:6) = [13; 0; 0];
    y(7:9) = [0; 0; 0];
    y(10:13) = quat(deg2rad(18), [0; 1; 0]);
    
    tspan = [0, 20.0];
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
    
    x_t = [x_0; 0; 0]; % m
    v_t = [v_0; 0; 0]; % m/s
    
    while t < 5
        %% Guidance loop
        x_t = x_t + v_t * dt;
        
        N = 3;
        x_r = qrot(qconj(y(10:13)), x_t - y(1:3));
        v_r = qrot(qconj(y(10:13)), v_t) - y(4:6);
        omega = cross(x_r, v_r) / dot(x_r, x_r);
        acc = -N * norm(y(4:6)) / norm(x_r) * cross(x_r, omega);
        los_angle = rad2deg(asin(dot(qrot(qconj(y(10:13)), [0;0;1]), x_r/norm(x_r))));
        
        if los_angle > 20.0 || y(3) < -10.0
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
            %sp = max(min(acc(3), 50), -30);
            sp = -20;
            dy = dynamics(t, y, params);
            fz_fb = dy(6) - y(4)*y(8);
            
            if t < params.tb
                u_tvc_cmd = az_tvc_pid.output(sp, fz_fb, y(8), dt);
                %if ~all(Kaz_tvc)
                    res.cost = res.cost + 0.01*az_tvc_pid.error^2*dt;
                %end
            else
                u_tvc_cmd = 0;
            end
            u_ae_cmd = az_ae_pid.output(sp, fz_fb, y(8), dt);
            
            params.vanes_cmd(1) = u_tvc_cmd;
            params.deltas_cmd(1) = u_ae_cmd;
            
            %if ~all(Kaz_ae)
                res.cost = res.cost + 0.01*az_ae_pid.error^2*dt;
            %end
        end
        
        %% Run simulation for one timestep
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% If missile loses control, stop the simulation and penalize cost value
        if norm(y(7:9)) > 200.0
             res.cost = res.cost + 1000;
             break;
        end
        
        %% Save current state in results array
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
        
        if y(3) >= 0
            break;
        end
    end
    
    %% Calculate miss distance
    idx = find(res.y(:,3) >= 0, 1);
    
    if length(idx) == 1
        x1 = res.y(idx-1,3);
        x2 = res.y(idx,3);
        f1 = res.y(idx-1,1:3)';
        f2 = res.y(idx,1:3)';
        
        xm_interp = f1 + (f2-f1)/(x2-x1)*(0-x1);
        miss_distance = norm(x_t - xm_interp);
        res.cost = res.cost + miss_distance;
        res.miss_distance = miss_distance;
    else
        miss_distance = norm(x_r);
        res.cost = res.cost + miss_distance;
        res.miss_distance = miss_distance;
    end
    
    %% Calculate rise time, setting time and overshoot (we want to minimize the sum of them)
%     indices = res.t >= 6.0;
%     info = stepinfo(res.dy(indices,6) - res.y(indices,4).*res.y(indices,8), res.t(indices), sp);
%     if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
%         res.cost = res.cost + 20;
%     else
%         res.cost = res.cost + info.RiseTime + (info.SettlingTime - 6.0) + abs(info.Overshoot - 4.598791);
%     end
end