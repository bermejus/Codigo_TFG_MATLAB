% Función que devuelve el valor inmediatamente posterior al dato de entrada
% según las tablas de valores que se hayan pasado a la función.
% Más rápida que la que implementa Matlab por defecto, velocidad de
% ejecución necesaria para ejecutar la simulación en un tiempo razonable.
% Ejemplo: xs = [0, 1, 2, 3]
%          ys = [4, 5, 6, 7]
%          x = 1.7
% La función devolverá y=6 en este caso.

function y = next(xs, ys, x)
    for i=2:length(xs)
        if x <= xs(i)
            y = ys(i);
            break;
        end
    end
end