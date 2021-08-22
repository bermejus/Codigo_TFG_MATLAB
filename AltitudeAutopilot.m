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
            obj.int_accum = obj.int_accum + 0.5 * (obj.error + obj.prev_error) * dt;
            u1 = obj.K(1) * obj.error;
            u2 = obj.K(2) * obj.error + obj.K(3) * obj.int_accum + obj.K(4) * (obj.error - obj.prev_error)/dt;
            obj.prev_error = obj.error;
            u_cmd = max(min([u1, u2], obj.u_max), obj.u_min);
        end
    end
end