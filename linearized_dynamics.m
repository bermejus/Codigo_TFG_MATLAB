function [A, B] = linearized_dynamics(t, y0, params)
    A = zeros(13,13);
    B = zeros(13,7);
    dx = 1e-7;
    dy0 = dynamics(t,y0,params);
    
    for i=1:13
        y = y0;
        y(i) = y(i) + dx;
        dy = dynamics(t,y,params);
        
        for j=1:13
            A(j,i) = (dy(j) - dy0(j)) / dx;
        end
    end
    
    p = params;
    for i=1:4
        p.vanes = params.vanes;
        p.vanes(i) = p.vanes(i) + dx;
        dy = dynamics(t,y0,p);
        
        for j=1:13
            B(j,i) = (dy(j) - dy0(j)) / dx;
        end
    end
    
    p.vanes = params.vanes;
    for i=1:3
        p.deltas = params.deltas;
        p.deltas(i) = p.deltas(i) + dx;
        dy = dynamics(t,y0,p);
        
        for j=1:13
            B(j,4+i) = (dy(j) - dy0(j)) / dx;
        end
    end
end