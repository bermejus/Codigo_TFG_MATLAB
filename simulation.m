%% Función que calcula la simulación completa del misil hasta que alcanza el blanco fijado.
% *** Argumentos de entrada:
% params: Parámetros del Javelin.
% Kh: Ganancias calculadas para el autopiloto en altura.
% Kaz: Ganancias calculadas para el autopiloto en aceleración.
% x_0: Posición inicial del blanco respecto al misil (m).
% v_0: Velocidad inicial del blanco (m/s).
%
% *** Devuelve una estructura con los siguientes contenidos:
% t: Vector que contiene los instantes de tiempo.
% y: Vector que contiene cada estado del msisil simulado en su correspondiente instante de tiempo.
% dy: Vector que contiene la salida de la función dynamics. Útil para graficar la aceleración.
% sp: Vector que contiene los setpoints generados en la etapa de guiado, útil para graficar aceleración.
% cost: Valor de la función de coste asociada a la simulación.
function res = simulation(params, Kh, Kaz, x_0, v_0)
    %% Condiciones iniciales
    y = zeros(17,1);
    y(1:3) = [0; 0; -1]; % Posición en ejes Tierra
    y(4:6) = [13; 0; 0]; % Velocidad en ejes cuerpo
    y(7:9) = [0; 0; 0]; % Velocidad angular
    y(10:13) = quat(deg2rad(18), [0; 1; 0]); % 18 grados de cabeceo
    
    %% Preparación de la simulación
    tspan = [0, 70.0];
    t = tspan(1);
    dt = 0.01; % Paso de tiempo, frecuencia de actualización 100Hz
    
    %% Inicializar variables de salida
    res.t = t;
    res.y = zeros(1,17);
    res.y(:) = y';
    res.dy = dynamics(t, y, params)';
    res.sp = 0.0;
    res.cost = 0;
    
    %% Autopiloto en altura
    altitude_pid = AltitudeAutopilot(Kh, deg2rad(20) * [-1,-1], deg2rad(20) * [1,1]);
    
    %% Autopiloto en aceleración
    az_tvc_pid = AccelAutopilot(Kaz(1:3), -deg2rad(20), deg2rad(20));
    az_ae_pid = AccelAutopilot(Kaz(4:6), -deg2rad(20), deg2rad(20));
    guidance_activated = false;
    
    %% Posición y velocidad del objetivo
    x_t = [x_0; 0; 0]; % m
    v_t = [v_0; 0; 0]; % m/s
    
    %% Diseño del criterio en altura para cambiar a la fase terminal de guiado
    if x_0 <= 825
        x = [150, 500, 750, 825];
        h = [10, 20, 25, 40];
        h_lim = linear(x, h, x_0);
    else
        h_lim = Inf;
    end
    
    %% Diseño de la máxima aceleración comandada en función de la posición inicial del objetivo
    x_clamp = min(max(x_0, 150), 1000);
    sp_max = linear([150, 250, 750, 1000], [30, 35, 40, 50], x_clamp);
    
    %% Comienzo de la simulación
    while t < tspan(2)
        %% Navegación proporcional con N=3
        x_t = x_t + v_t * dt; % Blanco se desplaza a v_t constante
        
        N = 3;
        x_r = qrot(qconj(y(10:13)), x_t - y(1:3));
        v_r = qrot(qconj(y(10:13)), v_t) - y(4:6);
        omega = cross(x_r, v_r) / dot(x_r, x_r);
        acc = -N * norm(y(4:6)) / norm(x_r) * cross(x_r, omega);
        los_angle = rad2deg(asin(dot(qrot(qconj(y(10:13)), [0;0;1]), x_r/norm(x_r))));
        
        % Criterios para activar la etapa de guiado
        if los_angle > 20 || y(3) < -h_lim
            guidance_activated = true;
        end
        
        if ~guidance_activated
            %% Control de altura
            sp_h = -150;
            u_cmd = altitude_pid.output(1.0, y(3)/sp_h, dt);
            if t < 2.0
                params.vanes_cmd(1) = u_cmd(1);
            else
                params.vanes_cmd(1) = 0;
            end
            params.deltas_cmd(1) = u_cmd(2);
            
            % Hasta que no se active el guiado se almacena sp=0.
            res.sp = [res.sp 0.0];
        else
            %% Control de aceleración
            sp = max(min(acc(3), sp_max), -10);
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
            
            % Actualizar valor de la función de coste para optimizar el
            % rendimiento del autopiloto en aceleración, penalizar la
            % desviación del setpoint.
            res.cost = res.cost + 0.005 * az_ae_pid.error^2 * dt;
            
            % Almacenar el setpoint generado en el vector de salida.
            res.sp = [res.sp sp];
        end
        
        %% Ejecutar la simulación para un paso de tiempo
        [ts, ys] = ode45(@(t,y) dynamics(t,y,params), [t, t+dt], y);
        t = ts(end);
        y = ys(end,:)';
        
        %% Si el misil pierde el control, finalizar la simulación y penalizar el evento
        if norm(y(7:9)) > 100.0
             res.cost = res.cost + 1000;
             break;
        end
        
        %% Guardar el estado actual en los vectores de salida
        res.t = [res.t; t];
        res.y = [res.y; y'];
        res.dy = [res.dy; dynamics(t, y, params)'];
        
        %% Si el misil se encuentra a una altura menor de 0 metros, finalizar la simulación
        if y(3) >= 0
            break;
        end
    end
    
    %% Calcular el error de alcance
    idx = find(res.y(:,3) >= 0, 1);
    
    if length(idx) == 1
        x1 = res.y(idx-1,3);
        x2 = res.y(idx,3);
        f1 = res.y(idx-1,1:3)';
        f2 = res.y(idx,1:3)';
        
        % Interpolación para encontrar altura nula estimada (la simulación
        % no para exactamente en h=0, por lo que se hace una interpolación
        % lineal para calcular el error de alcance).
        xm_interp = f1 + (f2-f1)/(x2-x1)*(0-x1);
        miss_distance = norm(x_t - xm_interp);
        res.cost = res.cost + miss_distance;
        res.miss_distance = miss_distance;
    else
        miss_distance = norm(x_r);
        res.cost = res.cost + miss_distance;
        res.miss_distance = miss_distance;
    end
end