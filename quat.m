function res = quat(angle, axis)
    res = [cos(angle/2); axis * sin(angle/2)];
end