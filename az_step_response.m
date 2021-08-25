%% Función que calcula la respuesta del autopiloto en aceleración a una entrada escalón.
% *** Argumentos de entrada:
% params: Parámetros del Javelin.
% Kh: Ganancias calculadas para el autopiloto en altura.
% Kaz: Ganancias calculadas para el autopiloto en aceleración.
% sp: Magnitud de la entrada escalón a aplicar (setpoint).
% t_trigger: Instante en el que se produce la entrada escalón.
% t_end: Duración deseada de la simulación.
%
% *** Devuelve una estructura con los siguientes contenidos:
% t: Vector que contiene los instantes de tiempo.
% y: Vector que contiene cada estado del msisil simulado en su correspondiente instante de tiempo.
% dy: Vector que contiene la salida de la función dynamics. Útil para graficar la aceleración.
% sp: Vector que contiene los setpoints de aceleración comandados, útil para graficar aceleración.
% RiseTime: Tiempo de subida ante la entrada escalón.
% SettlingTime: Tiempo de establecimiento ante la entrada escalón.
% Overshoot: Sobreimpulso ante la entrada escalón.
% damp: Amortiguamiento del autopiloto ante la entrada escalón.
% natural_freq: Frecuencia natural del autopiloto ante la entrada escalón.
function res = az_step_response(params, Kh, Kaz, sp, t_trigger, t_end)
    %% Condiciones iniciales
    y = zeros(17,1);
    y(1:3) = [0; 0; -1]; % Posición en ejes Tierra
    y(4:6) = [13; 0; 0]; % Velocidad en ejes cuerpo
    y(7:9) = [0; 0; 0]; % Velocidad angular
    y(10:13) = quat(deg2rad(18), [0; 1; 0]); % 18 grados de cabeceo
    
    %% Preparación de la simulación
    if ~exist("t_end", "var")
        t_end = 20.0;
    end
    
    tspan = [0, t_end];
    t = tspan(1);
    dt = 0.01; % Paso de tiempo, frecuencia de actualización 100Hz
    
    %% Inicializar variables de salida
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.dy = dynamics(t, y, params)';
    res.sp = 0.0;
    
    %% Autopiloto en altura
    altitude_pid = AltitudeAutopilot(Kh, deg2rad(20) * [-1,-1], deg2rad(20) * [1,1]);
    
    %% Autopiloto en aceleración
    az_tvc_pid = AccelAutopilot(Kaz(1:3), -deg2rad(20), deg2rad(20));
    az_ae_pid = AccelAutopilot(Kaz(4:6), -deg2rad(20), deg2rad(20));
    guidance_activated = false;
    
    %% Comienzo de la simulación
    while t < tspan(2)
        %% Control de aceleración (en caso de que se haya activado)
        if t >= t_trigger
            guidance_activated = true;
        end

        if ~guidance_activated
            % Control de altura (hasta que se active el guiado)
            sp_h = -150;
            u_cmd = altitude_pid.output(1.0, y(3)/sp_h, dt);
            if t < 2.0
                params.vanes_cmd(1) = u_cmd(1);
            else
                params.vanes_cmd(1) = 0;
            end
            params.deltas_cmd(1) = u_cmd(2);
            
            % Hasta que no se active el control de aceleración se almacena sp=0.
            res.sp = [res.sp 0.0];
        else
            % Control de aceleración
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
            
            % Almacenar el setpoint de entrada en el vector de salida.
            res.sp = [res.sp sp];
        end
        
        %% Ejecutar la simulación para un paso de tiempo
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% Si el misil pierde el control, finalizar la simulación
        if norm(y(7:9)) > 100.0
             break;
        end
        
        %% Guardar el estado actual en los vectores de salida
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
    end
    
    %% Cálculo de varios parámetros para entrada escalón, asumiendo que se aproxima a un sistema de segundo orden
    indices = res.t >= t_trigger;
    ts = res.t(indices);
    ys = res.dy(indices,6) - res.y(indices,4).*res.y(indices,8);
    % Importante: necesario instalar el paquete Signal Processing Toolbox.
    info = stepinfo(ys, ts, sp);
    
    res.RiseTime = info.RiseTime;
    res.SettlingTime = (info.SettlingTime - t_trigger);
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
    
    % Mostrar información calculada en consola
    fprintf("Tiempo de subida: %g s\n", res.RiseTime);
    fprintf("Tiempo de establecimiento: %g s\n", res.SettlingTime);
    fprintf("Sobreimpulso: %g %c\n", res.Overshoot, "%");
    fprintf("Amortiguamiento: %g\n", res.damp);
    fprintf("Frecuencia natural: %g Hz\n", res.natural_freq);
end