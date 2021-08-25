function dy = dynamics(t, y, p)
    %% Construir parámetros del misil en tiempo 't' mediante interpolación
    if t >= p.tb
        m = p.mass_lut(end);
        T = 0.0;
        cg = p.cg_lut(end);
        Ixx = p.Ixx_lut(end);
        Iyy = p.Iyy_lut(end);
        Ixx_dot = 0.0;
        Iyy_dot = 0.0;
    else
        m = linear(p.time_lut, p.mass_lut, t);
        T = linear(p.time_lut, p.thrust_lut, t);
        cg = linear(p.time_lut, p.cg_lut, t);
        Ixx = linear(p.time_lut, p.Ixx_lut, t);
        Iyy = linear(p.time_lut, p.Iyy_lut, t);
        Ixx_dot = next(p.time_lut, p.Ixx_dot_lut, t);
        Iyy_dot = next(p.time_lut, p.Iyy_dot_lut, t);
    end
    
    I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Iyy];
    Iinv = [1/Ixx, 0, 0; 0, 1/Iyy, 0; 0, 0, 1/Iyy];
    Idot = [Ixx_dot, 0, 0; 0, Iyy_dot, 0; 0, 0, Iyy_dot];
    
    %% Cálculo de fuerzas de empuje.
    forces = [0; 0; 0];
    moments = [0; 0; 0];
    
    % Transformación de los tres actuadores virtuales (elevador, alerones
    % y rudder) a los cuatro actuadores reales en configuración cruciforme.
    vanes = [-y(14) - p.vanes_cmd(2) - p.vanes_cmd(3);
             y(14) - p.vanes_cmd(2) - p.vanes_cmd(3);
             y(14) + p.vanes_cmd(2) - p.vanes_cmd(3);
             -y(14) + p.vanes_cmd(2) - p.vanes_cmd(3)];
    
    for i = 1:4
        phi = pi/4 + pi/2 * (i-1);
        thrust = [cos(vanes(i)); sin(phi)*sin(vanes(i)); -cos(phi)*sin(vanes(i))] * (T/4);
        forces = forces + thrust;
        moments = moments + cross([cg - p.l; cos(phi)*p.d/2; sin(phi)*p.d/2], thrust);
    end
    
    %% Cálculo de fuerzas y momentos aerodinámicos.
    V = norm(y(4:6));
    mach = V/p.a;
    
    % Ángulos de ataque y resbalamiento
    alpha = atan2(y(6), y(4));
    beta = atan2(y(5), V);
    
    % Interpolación bilineal para ángulo de ataque y Mach actuales
    CN = bilinear(p.alpha_lut, p.mach_lut, p.CN_lut, alpha, mach);
    CNq = bilinear(p.alpha_lut, p.mach_lut, p.CNq_lut, alpha, mach);
    CA = bilinear(p.alpha_lut, p.mach_lut, p.CA_lut, alpha, mach);
    CAq = bilinear(p.alpha_lut, p.mach_lut, p.CAq_lut, alpha, mach);
    CYb = bilinear(p.alpha_lut, p.mach_lut, p.CYb_lut, alpha, mach);
    CYp = bilinear(p.alpha_lut, p.mach_lut, p.CYp_lut, alpha, mach);
    CYr = bilinear(p.alpha_lut, p.mach_lut, p.CYr_lut, alpha, mach);
    Cm = bilinear(p.alpha_lut, p.mach_lut, p.Cm_lut, alpha, mach);
    Cmq = bilinear(p.alpha_lut, p.mach_lut, p.Cmq_lut, alpha, mach);
    Clb = bilinear(p.alpha_lut, p.mach_lut, p.Clb_lut, alpha, mach);
    Clp = bilinear(p.alpha_lut, p.mach_lut, p.Clp_lut, alpha, mach);
    Clr = bilinear(p.alpha_lut, p.mach_lut, p.Clr_lut, alpha, mach);
    Cnb = bilinear(p.alpha_lut, p.mach_lut, p.Cnb_lut, alpha, mach);
    Cnp = bilinear(p.alpha_lut, p.mach_lut, p.Cnp_lut, alpha, mach);
    Cnr = bilinear(p.alpha_lut, p.mach_lut, p.Cnr_lut, alpha, mach);
    
    % Interpolación lineal con Mach para coeficientes de actuadores
    CNde = linear(p.mach_lut, p.CNde, mach);
    Cmde = linear(p.mach_lut, p.Cmde, mach);
    CYdr = linear(p.mach_lut, p.CYdr, mach);
    Cndr = linear(p.mach_lut, p.Cndr, mach);
    Clda = linear(p.mach_lut, p.Clda, mach);
    
    qS = 0.5 * p.rho * V^2 * p.Sref;
    lon_adim = 2*p.c/V;
    lat_adim = 2*p.b/V;
    
    % Cálculo de fuerzas estacionarias y dinámicas
    Fae = -qS * [CA + CAq * lon_adim * y(8);
                 CYb * beta + CYp * lat_adim * y(7) + CYr * lat_adim * y(9) + CYdr * p.deltas_cmd(2);
                 CN + CNq * lon_adim * y(8) + CNde * y(16)];
    
    % Cálculo de momentos estacionarios y dinámicos aplicados en p.x0
    % (centro de masas al finalizar la combustión).
    Mae = qS * [p.b * (Clb * beta + lat_adim * (Clp * y(7) + Clr * y(9)) + Clda * p.deltas_cmd(3));
                p.c * (Cm + Cmq * lon_adim * y(8) + Cmde * y(16));
                p.b * (Cnb * beta + lat_adim * (Cnp * y(7) + Cnr * y(9)) + Cndr * p.deltas_cmd(2))];
    
    % Momento aerodinámico respecto al centro de masas en el instante 't'
    Mae = Mae + (cg - p.x0) * [0.0; -Fae(3); Fae(2)];
    
    %% Contribución del peso del misil en ejes cuerpo
    forces = forces + Fae + qrot(qconj(y(10:13)), [0.0; 0.0; m*p.g]);
    moments = moments + Mae;
    
    %% Cálculo de la dinámica del misil mediante la función no lineal: x_dot = f(x,u,t)
    dy = zeros(17,1);
    
    % Derivada de la posición en ejes Tierra
    dy(1:3) = qrot(y(10:13), y(4:6));
    
    % Aceleración en ejes cuerpo (SR no inercial)
    dy(4:6) = forces/m - cross(y(7:9), y(4:6));
    
    % Aceleración angular en ejes cuerpo (SR no inercial)
    dy(7:9) = Iinv * (moments - Idot*y(7:9) - cross(y(7:9), I*y(7:9)));
    
    % Derivada del cuaternión de orientación
    dy(10:13) = 0.5 * qmul(y(10:13), [0; y(7:9)]);
    
    % Elevador del empuje vectorizado, sistema lineal de segundo orden
    dy(14:15) = [0 1; -p.wn_te^2 -2*p.damp_te*p.wn_te] * y(14:15) + [0; p.wn_te^2] * p.vanes_cmd(1);
    
    % Elevador de las aletas, sistema lineal de segundo orden
    dy(16:17) = [0 1; -p.wn_e^2 -2*p.damp_e*p.wn_e] * y(16:17) + [0; p.wn_e^2] * p.deltas_cmd(1);
end