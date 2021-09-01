%% Construye una lista de parámetros del Javelin necesarios para la simulación.
params = parameters();

%% Cálculo de ganancias para autopiloto en altura
% Utiliza el algoritmo Surrogate Optimization, propio del paquete Global
% Optimization Toolbox de MATLAB, que minimiza la función de coste dada
% para encontrar el conjunto de constantes que definen el autopiloto en
% altura (PID), cuya respuesta es la buscada.

sp = -150.0; % setpoint (en metros, negativo por el sistema de coordenadas)
ub = [0.0, 1.8, 0.1, 1.8]; % Límites superiores para ganancias
lb = [-0.01, 0.0, 0.0, 0.0]; % Límites inferiores para ganancias

% Ejecutar el algoritmo en paralelo para utilizar todos los núcleos del
% procesador, necesario el paquete Parallel Computing Toolbox.
options = optimoptions('surrogateopt','UseParallel',true);

% Descomentar la línea siguiente para ejecutar el algoritmo, y comentar la
% posterior para evitar que se sobrescriban las ganancias.
%PID = surrogateopt(@(x) tune_altitude_autopilot(params, x, sp).cost, lb, ub, options);
PID = [-0.00486021630369243,1.30571708244770,0.0277657247170192,1.80000000000000];

% Mostrar las ganancias en la consola.
fprintf("TVC -> [Kp: %g], Fins -> [Kp: %g, Ki: %g, Kd: %g]\n", PID);

%% Respuesta del autopiloto en altura
% Diferentes gráficas que muestran la respuesta y desempeño del autopiloto
% en altura desarrolado. El último argumento es opcional, y pasándole valor
% 'true' muestra en consola tiempo de subida y establecimiento,
% sobreimpulso, amortiguamiento y frecuencia natural no amortiguada para
% esa respuesta en concreto.
res = tune_altitude_autopilot(params, PID, sp, true);

figure;
plot(res.t, -res.y(:,3));
title("Altura del misil");
xlabel("Tiempo (s)");
ylabel("Altura (m)");
grid();

euler = euler_angles(res.y(:,10:13));
figure;
plot(res.t, rad2deg(euler(:,2)));
title("Ángulo de cabeceo");
xlabel("Tiempo (s)");
ylabel("\theta (\circ)");
grid();

figure;
plot(res.t, rad2deg(atan2(res.y(:,6), res.y(:,4))));
title("Ángulo de ataque");
xlabel("Tiempo (s)");
ylabel("\alpha (\circ)");
grid();

figure;
plot(res.t, rad2deg(res.y(:,14)));
hold on;
plot(res.t, rad2deg(res.y(:,16)));
hold off;
title("Respuesta de los actuadores");
xlabel("Tiempo (s)");
ylabel("Elevador (\circ)");
legend(["{\delta}_{tvc}", "{\delta}_{ae}"]);
grid();

figure;
plot(res.t, vecnorm(res.y(:,4:6),2,2));
title("Velocidad del misil");
xlabel("Tiempo (s)");
ylabel("V (m/s)");
grid();

figure;
plot(res.y(:,1), -res.y(:,3));
title("Altura frente a alcance");
xlabel("Alcance (m)");
ylabel("Altura (m)");
grid();

%% Cálculo de ganancias para autopiloto en aceleración
ub = [0.0, 0.0, 0.1, 0.0, 0.0, 0.1]; % Límites superiores para ganancias
lb = [-0.01, -0.15, 0.0, 0.0, -0.1, 0.0]; % Límites inferiores para ganancias

% Mismo comando para ejecutar el algoritmo en paralelo, además de aumentar
% el número maximo de iteraciones a 2500. De todas formas, el algoritmo se
% puede parar en cualquier momento dando al botón Stop que aparece en la
% ventana emergente, además de mostrar la evolución del coste asociado a
% cada solución conforme aumenta el número de iteraciones.
options = optimoptions('surrogateopt','UseParallel',true,'MaxFunctionEvaluations',2500);

% Descomentar la línea siguiente para ejecutar el optimizador, y comentar
% la posterior para evitar sobrescribir la solución.
%Kaz = surrogateopt(@(x) guidance_optimizer(params, PID, x), lb, ub, options);
Kaz = [-0.00954961505678904,-0.0934648264030066,0.0971967283420569,0,-0.0820097950653736,0.0581924493489259];

% Mostrar las ganancias en la consola.
fprintf("Kaz tvc: [%g, %g, %g]\n", Kaz(1:3));
fprintf("Kaz ae: [%g, %g, %g]\n", Kaz(4:6));

% Diferentes simulaciones que muestran en consola el error de alcance para
% diferentes posiciones y velocidades iniciales del objetivo.
fprintf("Error de alcance para objetivo a 150m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 150, 0).miss_distance);
fprintf("Error de alcance para objetivo a 150m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 150, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 250m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 250, 0).miss_distance);
fprintf("Error de alcance para objetivo a 250m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 250, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 500m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 500, 0).miss_distance);
fprintf("Error de alcance para objetivo a 500m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 500, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 750m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 750, 0).miss_distance);
fprintf("Error de alcance para objetivo a 750m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 750, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 1000m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 1000, 0).miss_distance);
fprintf("Error de alcance para objetivo a 1000m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 1000, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 1500m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 1500, 0).miss_distance);
fprintf("Error de alcance para objetivo a 1500m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 1500, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 2000m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 2000, 0).miss_distance);
fprintf("Error de alcance para objetivo a 2000m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 2000, 40/3.6).miss_distance);
fprintf("Error de alcance para objetivo a 2500m, 0 km/h: %g m\n", simulation(params, PID, Kaz, 2500, 0).miss_distance);
fprintf("Error de alcance para objetivo a 2500m, 40 km/h: %g m\n", simulation(params, PID, Kaz, 2500, 40/3.6).miss_distance);

%% Simulación completa del misil con autopiloto en altura y etapa de guiado
% Opción 1: Mostrar gráficas para el vuelo completo, con un objetivo en la
% posición y velocidad deseadas.
res = simulation(params, PID, Kaz, 2500, 0/3.6);
res.miss_distance

% Opción 2: Respuesta del autopiloto en aceleración ante una entrada
% escalón con la magnitud especificada, tiempo en el que se aplica y
% duración de la simulación.
%res = az_step_response(params, PID, Kaz, 50, 7.0, 10.0);

figure;
plot(res.y(:,1), -res.y(:,3));
title("Altura frente a alcance");
xlabel("Alcance (m)");
ylabel("Altura (m)");
grid();

figure;
plot(res.t, -res.y(:,3));
title("Altura del misil");
xlabel("Tiempo (s)");
ylabel("Altura (m)");
grid();

euler = euler_angles(res.y(:,10:13));
figure;
plot(res.t, rad2deg(euler(:,2)));
title("Ángulo de cabeceo");
xlabel("Tiempo (s)");
ylabel("\theta (\circ)");
grid();

figure;
plot(res.t, rad2deg(atan2(res.y(:,6), res.y(:,4))));
title("Ángulo de ataque");
xlabel("Tiempo (s)");
ylabel("\alpha (\circ)");
grid();

figure;
plot(res.t, vecnorm(res.y(:,4:6),2,2));
title("Velocidad del misil");
xlabel("Tiempo (s)");
ylabel("V (m/s)");
grid();

figure;
plot(res.t, res.dy(:,6) - res.y(:,4).*res.y(:,8));
hold on;
plot(res.t, res.sp);
hold off;
title("Aceleración");
xlabel("Tiempo (s)");
ylabel("a_z (m/{s^2})");
legend(["a_z", "sp"]);
grid();

figure;
plot(res.t, rad2deg(res.y(:,14)));
hold on;
plot(res.t, rad2deg(res.y(:,16)));
hold off;
title("Respuesta de los actuadores");
xlabel("Tiempo (s)");
ylabel("Elevador (\circ)");
legend(["{\delta}_{tvc}", "{\delta}_{ae}"]);
grid();

%% Graficar la persecución del objetivo para distintas posiciones iniciales
% Las posiciones iniciales se distribuyen en varios puntos representativos
% del desempeño del misil, desde el alcance mínimo (150 metros) hasta el
% alcance máximo (2500 metros).

res1 = simulation(params, PID, Kaz, 150, 0/3.6);
res2 = simulation(params, PID, Kaz, 250, 0/3.6);
res3 = simulation(params, PID, Kaz, 500, 0/3.6);
res4 = simulation(params, PID, Kaz, 750, 0/3.6);
res5 = simulation(params, PID, Kaz, 1000, 0/3.6);
res6 = simulation(params, PID, Kaz, 1500, 0/3.6);
res7 = simulation(params, PID, Kaz, 2000, 0/3.6);
res8 = simulation(params, PID, Kaz, 2500, 0/3.6);

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
title("Persecución del objetivo");
xlabel("Alcance (m)");
ylabel("Altura (m)");
grid();

%% Funciones
% Función que calcula el valor de la función de coste con las ganancias
% elegidas para la etapa de guiado. Devuelve la suma de los costes para una
% posición inicial de 150, 500 y 2000 metros y velocidades iniciales de 0,
% 20 y 40 km/h.
function cost = guidance_optimizer(params, PID, Kaz)
    cost = 0;
    
    % Optimizar para una posición inicial del objetivo de entre 150 a 2000
    % metros para evitar que encuentre únicamente una solución local, pero
    % que no sirva para todo el rango operativo del misil.
    
    sp = [150, 500, 2000];
    v = [0, 20/3.6, 40/3.6];
    
    for i=1:length(sp)
        for j=1:length(v)
            cost = cost + simulation(params, PID, Kaz, sp(i), v(j)).cost;
        end
    end
end