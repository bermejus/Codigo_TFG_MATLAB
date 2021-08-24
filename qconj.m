% Función que calcula el cuaternión conjugado del cuaternión dado como
% argumento.
function res = qconj(q)
    res = [q(1); -q(2); -q(3); -q(4)];
end