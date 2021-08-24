% Función que rota un vector según el cuaternión de rotación dado como
% argumento. Versión optimizada de la siguiente operación: q*v*q'
% Nota: q' representa el cuaternión conjugado.
function res = qrot(q, v)
    t = 2 * cross(q(2:4), v);
    res = v + q(1) * t + cross(q(2:4), t);
end