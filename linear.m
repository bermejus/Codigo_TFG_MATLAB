% Función que realiza una interpolación lineal con los datos de entrada.
% Más rápida que la que implementa Matlab por defecto, velocidad de
% ejecución necesaria para ejecutar la simulación en un tiempo razonable.
function y = linear(xs, ys, x)
    idx = 1;
    for i=2:length(xs)
        if x <= xs(i)
            idx = i-1;
            break;
        end
    end
    
    a = x - xs(idx);
    b = xs(idx+1) - x;
    
    y = (b*ys(idx) + a*ys(idx+1)) / (b + a);
end