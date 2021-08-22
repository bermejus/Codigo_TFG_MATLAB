classdef Az_lqr < handle
    properties
        error
        epsilon
        Kv
        Q
        R
        u_min
        u_max
    end
    
    methods
        function obj = Az_lqr(Kv, u_min, u_max)
            obj.error = 0;
            obj.epsilon = 0;
            obj.Kv = Kv;
            obj.Q = [Kv(1) 0; 0 Kv(2)];
            obj.R = [Kv(3) 0; 0 Kv(4)];
            obj.u_min = u_min;
            obj.u_max = u_max;
        end
        
        function res = output(obj, t, y, p, sp, dt)
            %% Find coefficients
            if t < p.tb
                m = linear(p.time_lut, p.mass_lut, t);
                T = linear(p.time_lut, p.thrust_lut, t);
                Iyy = linear(p.time_lut, p.Iyy_lut, t);
                cg = linear(p.time_lut, p.cg_lut, t);
            else
                m = p.mass_lut(end);
                T = 0;
                Iyy = p.Iyy_lut(end);
                cg = p.cg_lut(end);
            end

            V = norm(y(4:6));
            mach = V/p.a;
            CNa = bilinear(p.alpha_lut, p.mach_lut, p.CNa_lut, 0, mach);
            CNq = bilinear(p.alpha_lut, p.mach_lut, p.CNq_lut, 0, mach);
            CNde = linear(p.mach_lut, p.CNde, mach);
            Cma = bilinear(p.alpha_lut, p.mach_lut, p.Cma_lut, 0, mach);
            Cmq = bilinear(p.alpha_lut, p.mach_lut, p.Cmq_lut, 0, mach);
            Cmde = linear(p.mach_lut, p.Cmde, mach);
            
            %% Build matrices
            qinf = 0.5 * p.rho * V^2;
            a = -qinf * p.Sref * CNa / (m * y(4));
            b = -qinf * p.Sref * CNq * (2 * p.c / V) / m + y(4);
            c = -qinf * p.Sref * CNde / m;
            d = T / (sqrt(2) * m);

            e = qinf * p.Sref * p.c * Cma / (Iyy * y(4));
            f = qinf * p.Sref * p.c * Cmq * (2 * p.c / V) / Iyy;
            g = qinf * p.Sref * p.c * Cmde / Iyy;
            h = (p.l - cg) * T / (sqrt(2) * Iyy);
            
            A = [a b; e f];
            B = [c d; g h];
            
            %% Calculate gain matrix and actuator commands
            dy = dynamics(t, y, p);
            obj.error = dy(6) - sp;
            obj.epsilon = obj.epsilon + obj.error * dt;
            
            K = lqr(A, B, obj.Q, obj.R);
            u = -K * [y(6); y(8)] + [obj.Kv(5); obj.Kv(6)] * obj.epsilon;
            res = max(min(u, obj.u_max), obj.u_min);
        end
    end
end