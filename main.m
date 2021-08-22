%% Build parameter list
params = parameters();

%% Calculate height autopilot gains
sp = -150.0;
ub = [0.01, 5.0, 0.1, 5.0];
lb = [0.0, 2.0, 0.01, 3];

options = optimoptions('surrogateopt','UseParallel',true);
%PID = surrogateopt(@(x) tune_altitude_autopilot(params, x, sp).cost, lb, ub, options);
PID = [0.00749222349359639,3.71135089747078,0.0357381908067924,3.35647278501200];
fprintf("TVC -> [Kp: %g], Fins -> [Kp: %g, Ki: %g, Kd: %g]\n", PID);

res = tune_altitude_autopilot(params, PID, sp);

%% Calculate second order system parameters
info = stepinfo(-res.y(:,3), res.t, -sp);
info.RiseTime
info.SettlingTime
info.Overshoot

%% Graph relevant variables
figure;
plot(res.t, -res.y(:,3));
title("Altitud del misil");
xlabel("Tiempo (s)");
ylabel("Altitud (m)");
grid();

euler = euler_angles(res.y(:,10:13));
figure;
plot(res.t, rad2deg(euler(:,2)));
title("Pitch angle");
xlabel("Time (s)");
ylabel("Pitch (deg)");
grid();

figure;
plot(res.t, rad2deg(atan2(res.y(:,6), res.y(:,4))));
title("Angle of attack");
xlabel("Time (s)");
ylabel("AoA (deg)");
grid();

figure;
plot(res.t, rad2deg(res.y(:,14)));
hold on;
plot(res.t, rad2deg(res.y(:,16)));
hold off;
title("Deflection");
xlabel("Time (s)");
ylabel("Elevator (deg)");
legend(["tvc", "fins"]);
grid();

figure;
plot(res.t, vecnorm(res.y(:,4:6),2,2));
title("Velocity");
xlabel("Time (s)");
ylabel("Velocity (m/s)");
grid();

figure;
plot(res.y(:,1), -res.y(:,3));
title("Envolvente de vuelo");
xlabel("Alcance (m)");
ylabel("Altitud (m)");
grid();

%% Lets calculate this fucking shit - Low Range
ub = [0.0, 0.0, 0.1, 0.0, 0.0, 0.1];
lb = [-0.01, -0.15, 0.0, 0.0, -0.1, 0.0];

options = optimoptions('surrogateopt','UseParallel',true,'MaxFunctionEvaluations',5000);
%Kaz = surrogateopt(@(x) guidance_optimizer_low_range(params, PID, x(1:3), x(4:6)), lb, ub, options);
Kaz = [-0.00842833585481730,-0.0731870428563336,0.0402739030601024,0,-0.0720768623041320,0.0397369040927447];

fprintf("Kaz tvc: [%g, %g, %g]\n", Kaz(1:3));
fprintf("Kaz ae: [%g, %g, %g]\n", Kaz(4:6));

fprintf("Miss distance target at 150m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 150, 0/3.6).miss_distance);
fprintf("Miss distance target at 150m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 150, 40/3.6).miss_distance);
fprintf("Miss distance target at 250m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 250, 0/3.6).miss_distance);
fprintf("Miss distance target at 250m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 250, 40/3.6).miss_distance);
fprintf("Miss distance target at 500m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 500, 0/3.6).miss_distance);
fprintf("Miss distance target at 500m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 500, 40/3.6).miss_distance);
fprintf("Miss distance target at 750m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 750, 0/3.6).miss_distance);
fprintf("Miss distance target at 750m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 750, 40/3.6).miss_distance);
fprintf("Miss distance target at 1000m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1000, 0/3.6).miss_distance);
fprintf("Miss distance target at 1000m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1000, 40/3.6).miss_distance);
fprintf("Miss distance target at 1500m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1500, 0/3.6).miss_distance);
fprintf("Miss distance target at 1500m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1500, 40/3.6).miss_distance);
fprintf("Miss distance target at 2000m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2000, 0/3.6).miss_distance);
fprintf("Miss distance target at 2000m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2000, 40/3.6).miss_distance);
fprintf("Miss distance target at 2500m, 0 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2500, 0/3.6).miss_distance);
fprintf("Miss distance target at 2500m, 40 km/h: %g m\n", guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2500, 40/3.6).miss_distance);

res = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1500, 0/3.6);
%res = tune_az_autopilot_low_range(params, PID, Kaz(1:3), Kaz(4:6), 30);

figure;
plot(res.y(:,1), -res.y(:,3));
title("Envolvente de vuelo");
xlabel("Alcance (m)");
ylabel("Altitud (m)");
grid();

figure;
plot(res.t, -res.y(:,3));
title("Altitud del misil");
xlabel("Tiempo (s)");
ylabel("Altitud (m)");
grid();

euler = euler_angles(res.y(:,10:13));
figure;
plot(res.t, rad2deg(euler(:,2)));
title("Pitch angle");
xlabel("Time (s)");
ylabel("Pitch (deg)");
grid();

figure;
plot(res.t, rad2deg(atan2(res.y(:,6), res.y(:,4))));
title("Angle of attack");
xlabel("Time (s)");
ylabel("AoA (deg)");
grid();

figure;
plot(res.t, res.dy(:,6) - res.y(:,4).*res.y(:,8));
title("Aceleracion");
xlabel("Tiempo (s)");
ylabel("Aceleracion (m/s^2)");
grid();

figure;
plot(res.t, rad2deg(res.y(:,14)));
hold on;
plot(res.t, rad2deg(res.y(:,16)));
hold off;
title("Deflection");
xlabel("Time (s)");
ylabel("Elevator (deg)");
legend(["tvc", "fins"]);
grid();

%%
res1 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 150, 0/3.6);
res2 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 250, 0/3.6);
res3 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 500, 0/3.6);
res4 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 750, 0/3.6);
res5 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1000, 0/3.6);
res6 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 1500, 0/3.6);
res7 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2000, 0/3.6);
res8 = guidance_low_range(params, PID, Kaz(1:3), Kaz(4:6), 2500, 0/3.6);

figure;
plot(res1.y(:,1), -res1.y(:,3));
hold on;
plot(res2.y(:,1), -res2.y(:,3));
plot(res3.y(:,1), -res3.y(:,3));
plot(res4.y(:,1), -res4.y(:,3));
plot(res5.y(:,1), -res5.y(:,3));
plot(res6.y(:,1), -res6.y(:,3));
plot(res7.y(:,1), -res7.y(:,3));
plot(res8.y(:,1), -res8.y(:,3));
hold off;
title("Envolvente de vuelo");
xlabel("Alcance (m)");
ylabel("Altitud (m)");
grid();

%% Functions

function cost = guidance_optimizer_low_range(params, PID, Kaz_tvc, Kaz_ae)
    cost = 0;
    
    sp = [150];
    v = [0, 20/3.6, 40/3.6];
    
    for i=1:length(sp)
        for j=1:length(v)
            cost = cost + guidance_low_range(params, PID, Kaz_tvc, Kaz_ae, sp(i), v(j)).cost;
        end
    end
end