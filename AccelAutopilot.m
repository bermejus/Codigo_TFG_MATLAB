classdef AccelAutopilot < handle
    properties
        K
        error
        int_accum_1
        int_accum_2
        prev_error
        u_min
        u_max
    end
    methods
        function obj = AccelAutopilot(K, u_min, u_max)
            obj.K = K;
            obj.error = 0;
            obj.int_accum_1 = 0;
            obj.int_accum_2 = 0;
            obj.prev_error = 0;
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        function u_cmd = output(obj, sp, fz_fb, q_fb, dt)
            obj.error = sp - fz_fb;
            obj.int_accum_1 = obj.int_accum_1 + obj.error * dt;
            %u1 = obj.K(1) * obj.error + obj.K(2) * obj.int_accum_1;
            
            %e2 = u1 - q_fb;
            %obj.int_accum_2 = obj.int_accum_2 + e2 * dt;
            %u2 = obj.K(3) * e2 + obj.K(4) * obj.int_accum_2;
            
            u = obj.K(1) * obj.error + obj.K(2) * obj.int_accum_1 - obj.K(3) * q_fb;
            u_cmd = max(min(u, obj.u_max), obj.u_min);
            %u_cmd = max(min(u2, obj.u_max), obj.u_min);
        end
    end
end