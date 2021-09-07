w_n = 500 * 2*pi; % rad/s
damp = 0.7;

sys = tf(w_n^2, [1 2*damp*w_n, w_n^2]);

%% Step response
[y, t] = step(sys);

figure;
plot(t, y, 'LineWidth',1.25);
title("Step Response");
xlim([t(1) t(end)]);
xlabel("Time (seconds)");
ylabel("Amplitude");
grid;

info = stepinfo(y, t, 1.0);
fprintf("Rise Time: %g s.\n", info.RiseTime);
fprintf("Settling Time: %g s.\n", info.SettlingTime);
fprintf("Overshoot: %g %c.\n", info.Overshoot, "%");

%% Bode plot
bode(sys);