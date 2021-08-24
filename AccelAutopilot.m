%% Clase que implementa el autopiloto en aceleración.
% Se ha fijado el mismo diseño para el control vectorizado y control con
% aletas, siendo un controlador proporcional-integrador (PI) en aceleración
% con realimentación proporcional de velocidad angular de cabeceo 'q' para
% aumentar el amortiguamiento del conjunto.

classdef AccelAutopilot < handle
    properties
        K
        error
        int_accum
        u_min
        u_max
    end
    methods
        function obj = AccelAutopilot(K, u_min, u_max)
            obj.K = K;
            obj.error = 0;
            obj.int_accum = 0;
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        % 'fz_fb' es la aceleración en ejes cuerpo realimentada, y 'q_fb'
        % es la velocidad angular de cabeceo realimentada.
        function u_cmd = output(obj, sp, fz_fb, q_fb, dt)
            obj.error = sp - fz_fb;
            obj.int_accum = obj.int_accum + obj.error * dt;

            u = obj.K(1) * obj.error + obj.K(2) * obj.int_accum - obj.K(3) * q_fb;
            u_cmd = max(min(u, obj.u_max), obj.u_min);
        end
    end
end