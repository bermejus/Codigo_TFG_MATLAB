% Devuelve el cuaternión definido por el ángulo y el eje de rotación dados.
% Nota: el ángulo se da en radianes y el sentido de la rotación cumple la
% regla de la mano derecha. Matlab tiene funciones para operar con
% cuaterniones, pero el sentido de la rotación no es el esperado y resulta
% anti-intuitivo.
function res = quat(angle, axis)
    res = [cos(angle/2); axis * sin(angle/2)];
end