classdef AccelController < handle
    properties
        K
        error
        int_accum_1
        int_accum_2
        u_min
        u_max
    end
    methods
        function obj = AccelController(K, u_min, u_max)
            obj.K = K;
            obj.error = 0;
            obj.int_accum_1 = 0;
            obj.int_accum_2 = 0;
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        function cmd = output(obj, Az_cmd, Az_fb, q_fb, dt)
            obj.error = Az_cmd - Az_fb;
            u1 = obj.K(1) * obj.error + obj.K(2) * obj.int_accum_1;
            obj.int_accum_1 = obj.int_accum_1 + obj.error * dt;
            
            e2 = u1 - q_fb;
            u2 = obj.K(3) * e2 + obj.K(4) * obj.int_accum_2;
            obj.int_accum_2 = obj.int_accum_2 + e2 * dt;
            
            cmd = max(min(u2, obj.u_max), obj.u_min);
        end
    end
end