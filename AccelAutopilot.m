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
        
        function u_cmd = output(obj, sp, fz_fb, q_fb, dt)
            obj.error = sp - fz_fb;
            obj.int_accum = obj.int_accum + obj.error * dt;

            u = obj.K(1) * obj.error + obj.K(2) * obj.int_accum - obj.K(3) * q_fb;
            u_cmd = max(min(u, obj.u_max), obj.u_min);
        end
    end
end