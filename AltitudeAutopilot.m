%% Clase que implementa el autopiloto en altura.
% El diseño óptimo se ha fijado en control proporcional el empuje
% vectorizado y control proporcional-integrador-derivativo para las aletas.
% Permite limitar las deflexiones máximas tanto positivas como negativas
% mediante las variables 'u_max' y 'u_min', respectivamente.

classdef AltitudeAutopilot < handle
    properties
        K
        error
        int_accum
        prev_error
        u_min
        u_max
    end
    methods
        function obj = AltitudeAutopilot(K, u_min, u_max)
            obj.K = K;
            obj.error = 0;
            obj.int_accum = 0;
            obj.prev_error = 0;
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        function u_cmd = output(obj, sp, h_fb, dt)
            obj.error = sp - h_fb;
            
            % Término donde se acumula y calcula el valor de la integral
            % mediante la regla del trapecio.
            obj.int_accum = obj.int_accum + 0.5 * (obj.error + obj.prev_error) * dt;
            
            u_tvc = obj.K(1) * obj.error;
            u_ae = obj.K(2) * obj.error + obj.K(3) * obj.int_accum + obj.K(4) * (obj.error - obj.prev_error)/dt;
            obj.prev_error = obj.error;
            
            % Saturación de los comandos según valores máximos y mínimos.
            u_cmd = max(min([u_tvc, u_ae], obj.u_max), obj.u_min);
        end
    end
end