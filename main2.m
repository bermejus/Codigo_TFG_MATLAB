%% Build parameter list
params = parameters();

%% Run simulation
sp = -9.81;
ub = [0.0, 0.0 3.0 3.0, 0.0 0.0 0.0 0.0];
lb = [-0.01 -0.01 0.0 0.0, -0.01 -0.01 -3.0 -3.0];

options = optimoptions('surrogateopt','UseParallel',true);
PID = surrogateopt(@(x) tune_accel_autopilot(params, x, sp).cost, lb, ub, options);
PID

res = tune_accel_autopilot(params, PID, sp);

info = stepinfo(res.dy(:,6), res.t, sp);
info.RiseTime
info.SettlingTime
info.Overshoot

figure;
plot(res.t, -res.y(:,3));
title("Altitud del misil");
xlabel("Tiempo (s)");
ylabel("Altitud (m)");
grid();

figure;
plot(res.t, res.dy(:,6));
title("Aceleracion");
xlabel("Tiempo (s)");
ylabel("Aceleracion (m/s^2)");
grid();

euler = euler_angles(res.y(:,10:13));
figure;
plot(res.t, rad2deg(euler(:,2)));
title("Pitch angle");
xlabel("Time (s)");
ylabel("Pitch (deg)");
grid();

% figure;
% plot(res.t, res.y(:,8));
% title("Pitch rate");
% xlabel("Time (s)");
% ylabel("Pitch rate (rad/s)");
% grid();

% figure;
% plot(res.t, rad2deg(atan2(res.y(:,6), res.y(:,4))));
% title("Angle of attack");
% xlabel("Time (s)");
% ylabel("AoA (deg)");
% grid();

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

% figure;
% plot(res.t, rad2deg(res.y(:,15)));
% hold on;
% plot(res.t, rad2deg(res.y(:,17)));
% hold off;
% title("Deflection rate");
% xlabel("Time (s)");
% ylabel("Elevator rate (deg/s)");
% legend(["tvc", "fins"]);
% grid();

% figure;
% plot(res.t, vecnorm(res.y(:,4:6),2,2));
% title("Velocity");
% xlabel("Time (s)");
% ylabel("Velocity (m/s)");
% grid();
% 
% figure;
% plot(res.y(:,1), -res.y(:,3));
% title("Envolvente de vuelo");
% xlabel("Alcance (m)");
% ylabel("Altitud (m)");
% grid();