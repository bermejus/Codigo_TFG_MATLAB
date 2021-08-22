tspan = [0.0, 0.1];
y0 = [0.0; 0.0];

[ts, ys] = ode45(@(t,y) control(t,y,1), tspan, y0);

figure;
plot(ts, ys(:,1));
grid()

PID = AltitudeAutopilot([1, 2, 3, 4, 5], deg2rad(-15), deg2rad(15));
for i=1:10
    PID.output(10, 1, 1, 1, 1, 0.01);
end

function dy = control(t, y, cmd)
    wn = 2*pi*40.909;
    damp = 0.7;
    dy = [0 1; -wn^2 -2*damp*wn]*y + [0; wn^2]*cmd;
end