%% Función que calcula la respuesta del autopiloto en altura.
% *** Argumentos de entrada:
% params: Parámetros del Javelin.
% K: Ganancias calculadas para el autopiloto en altura.
% sp: Altitud deseada (setpoint).
% print_snd_order_info: si el valor es 'true', muestra en consola los parámetros de sistema de segundo orden.
%
% *** Devuelve una estructura con los siguientes contenidos:
% t: Vector que contiene los instantes de tiempo.
% y: Vector que contiene cada estado del msisil simulado en su correspondiente instante de tiempo.
% cost: Valor de la función de coste asociada a la simulación.
% RiseTime: Tiempo de subida ante la entrada escalón.
% SettlingTime: Tiempo de establecimiento ante la entrada escalón.
% Overshoot: Sobreimpulso ante la entrada escalón.
% damp: Amortiguamiento del autopiloto ante la entrada escalón.
% natural_freq: Frecuencia natural del autopiloto ante la entrada escalón.
function res = tune_altitude_autopilot(params, K, sp, print_snd_order_info)
    %% Condiciones iniciales
    y = zeros(17,1);
    y(1:3) = [0; 0; -1]; % Posición en ejes Tierra
    y(4:6) = [13; 0; 0]; % Velocidad en ejes cuerpo
    y(7:9) = [0; 0; 0]; % Velocidad angular
    y(10:13) = quat(deg2rad(18), [0; 1; 0]); % 18 grados de cabeceo
    
    %% Preparación de la simulación
    if ~exist("print_snd_order_info", "var")
        print_snd_order_info = false;
    end
    
    tspan = [0, 20];
    t = tspan(1);
    dt = 0.01; % Paso de tiempo, frecuencia de actualización 100Hz
    
    %% Inicializar variables de salida
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.cost = 0;
    
    %% Autopiloto en altura
    pid = AltitudeAutopilot(K, deg2rad(20) * [-1,-1], deg2rad(20) * [1,1]);
    
    %% Comienzo de la simulación
    while t < tspan(2)
        %% Control de altura a 150 metros
        u_cmd = pid.output(1.0, y(3)/sp, dt);
        if t < 2.0
            params.vanes_cmd(1) = u_cmd(1);
        else
            params.vanes_cmd(1) = 0;
        end
        params.deltas_cmd(1) = u_cmd(2);
        
        %% Ejecutar la simulación para un paso de tiempo
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% Si el misil pierde el control, finalizar la simulación y penalizar el evento
        if y(3) > 0 || norm(y(7:9)) > 100.0
             res.cost = res.cost + 30;
             break;
        end
        
        %% Guardar el estado actual en los vectores de salida
        res.t = [res.t; t];
        res.y = [res.y; y'];
        
        % Actualizar valor de la función de coste para optimizar el
        % rendimiento del autopiloto en altura, penalizar la desviación del
        % setpoint.
        res.cost = res.cost + 50*pid.error^2*dt;
    end
    
    %% Cálculo de varios parámetros para entrada escalón, asumiendo que se aproxima a un sistema de segundo orden
    ts = res.t;
    ys = -res.y(:,3);
    % Importante: necesario instalar el paquete Signal Processing Toolbox.
    info = stepinfo(ys, ts, -sp);
    
    res.RiseTime = info.RiseTime;
    res.SettlingTime = info.SettlingTime;
    res.Overshoot = info.Overshoot;
    if info.Overshoot == 0
        res.damp = 1.0;
        res.natural_freq = 0;
    elseif info.Overshoot > 100
        res.damp = NaN;
        res.natural_freq = NaN;
    else
        res.damp = fzero(@(x) (exp(-pi*x/sqrt(1-x^2)) - info.Overshoot/100), 0.5);
        res.natural_freq = (pi - acos(res.damp)) / (2*pi*info.RiseTime);
    end
    
    % Minimizar tiempo de subida, establecimiento y buscar coeficiente de
    % amortiguamiento lo más cercano a 0.7 posible. Si el sistema no se
    % corresponde con uno de segundo orden, penalizar aún más el valor de
    % la función de coste.
    if isnan(info.RiseTime) || isnan(info.SettlingTime) || isnan(info.Overshoot)
        res.cost = res.cost + 30;
    else
        res.cost = res.cost + info.RiseTime + info.SettlingTime + max(0.0, info.Overshoot - 3.0); % + 100 * abs(res.damp - 0.7);
    end
    
    % Mostrar información calculada en consola
    if print_snd_order_info
        fprintf("Tiempo de subida: %g s\n", res.RiseTime);
        fprintf("Tiempo de establecimiento: %g s\n", res.SettlingTime);
        fprintf("Sobreimpulso: %g %c\n", res.Overshoot, "%");
        fprintf("Amortiguamiento: %g\n", res.damp);
        fprintf("Frecuencia natural: %g Hz\n", res.natural_freq);
    end
end