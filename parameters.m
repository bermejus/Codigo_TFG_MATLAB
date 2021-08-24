function p = parameters()
    %% Cargar los datos aerodinámicos obtenidos con Missile DATCOM
    aero = datcomimport("for006.dat");

    %% Construir lista de parámetros necesarios para la simulación
    % Constantes generales varias
    p.a = 340.3; % Velocidad del sonido a nivel del mar (m/s)
    p.rho = 1.225; % Densidad del aire a nivel del mar (kg/m^3)
    p.g = 9.81; % Aceleración de la gravedad (m/s^2)

    p.l = 1.1; % Longitud del misil (m)
    p.d = 0.127; % Diámetro del misil (m)
    p.Sref = pi*(p.d/2)^2; % Superficie de referencia (m^2)
    p.c = 0.05; % Cuerda alar (m)
    p.b = 2*0.134; % Envergadura alar (m)
    p.x0 = 0.446; % Centro de masas al fin de combustión (m). En él se han calculado los coeficientes de momento.

    p.tb = 5.2; % Tiempo fin de combustión (s)
    p.time_lut = [0.0, 0.3, 0.6, 1.2, 1.8, 2.4, 4.2, 5.2]; % Tabla de tiempos (s)
    p.thrust_lut = [0.0, 570.0, 650.0, 750.0, 770.0, 650.0, 50.0, 0.0]; % Tabla de empuje (N)
    p.mass_lut = [11.25, 11.16, 11.06, 10.82, 10.58, 10.38, 10.16, 10.15]; % Tabla de la masa del misil (kg)
    p.cg_lut = [0.565, 0.555, 0.544, 0.519, 0.493, 0.471, 0.447, 0.446]; % Tabla del centro de gravedad (m)
    p.Ixx_lut = [0.0252, 0.0250, 0.0248, 0.0244, 0.0239, 0.0235, 0.0231, 0.0231]; % Momento de inercia en eje longitudinal (kg*m^2)
    p.Iyy_lut = [0.985, 0.979, 0.973, 0.958, 0.942, 0.929, 0.915, 0.914]; % Momento de inercia en ejes cuerpo 'y' y 'z' (kg*m^2)
    p.Ixx_dot_lut = [0.0, diff(p.Ixx_lut)./diff(p.time_lut)]; % Derivada de Ixx (kg*m^2/s)
    p.Iyy_dot_lut = [0.0, diff(p.Iyy_lut)./diff(p.time_lut)]; % Derivada de Iyy (kg*m^2/s)
    
    % Tablas con diferentes parámetros aerodinámicos, sus nombres son
    % autodescriptivos.
    % Los valores calculados para M=0.1 se asume que se mantienen
    % constantes para números de Mach menores que éste.
    p.alpha_lut = deg2rad(aero{1}.alpha); % Tabla de ángulos de ataque (grados)
    p.mach_lut = [0.0 aero{1}.mach]; % Tabla de números de Mach
    p.CN_lut = [aero{1}.cn(:,1) aero{1}.cn];
    p.CNa_lut = [aero{1}.cna(:,1) aero{1}.cna];
    p.CNq_lut = [aero{1}.cnq(:,1) aero{1}.cnq];
    p.CA_lut = [aero{1}.ca(:,1) aero{1}.ca];
    p.CAq_lut = [aero{1}.caq(:,1) aero{1}.caq];
    p.CYb_lut = [aero{1}.cyb(:,1) aero{1}.cyb];
    p.CYp_lut = [aero{1}.cyp(:,1) aero{1}.cyp];
    p.CYr_lut = [aero{1}.cyr(:,1) aero{1}.cyr];
    p.Cm_lut = [aero{1}.cm(:,1) aero{1}.cm];
    p.Cma_lut = [aero{1}.cma(:,1) aero{1}.cma];
    p.Cmq_lut = [aero{1}.cmq(:,1) aero{1}.cmq];
    p.Cl_lut = [aero{1}.cll(:,1) aero{1}.cll];
    p.Clb_lut = [aero{1}.clb(:,1) aero{1}.clb];
    p.Clp_lut = [aero{1}.clp(:,1) aero{1}.clp];
    p.Clr_lut = [aero{1}.clr(:,1) aero{1}.clr];
    p.Cn_lut = [aero{1}.cln(:,1) aero{1}.cln];
    p.Cnb_lut = [aero{1}.cnb(:,1) aero{1}.cnb];
    p.Cnp_lut = [aero{1}.cnp(:,1) aero{1}.cnp];
    p.Cnr_lut = [aero{1}.cnr(:,1) aero{1}.cnr];
    
    % Comandos enviados al control por empuje vectorizado.
    p.vanes_cmd = [0.0, 0.0, 0.0];
    % Comandos enviados al control por aletas.
    p.deltas_cmd = [0.0, 0.0, 0.0];
    % Nota: ambos en orden [elevador, alerones, rudder].
    
    % Amortiguamiento y frecuencias naturales para actuadores y sensores.
    % p.*_t[e,a,r] denota empuje vectorial, mientras que
    % p.*_[e,a,r] denota que aplica a aletas.
    p.damp_te = 0.7; % Amortiguamiento
    p.damp_ta = 0.7;
    p.damp_tr = 0.7;
    p.wn_te = 257.04; % Frecuencia natural (rad/s)
    p.wn_ta = 257.04; % (rad/s)
    p.wn_tr = 257.04; % (rad/s)
    
    p.damp_e = 0.7;
    p.damp_a = 0.7;
    p.damp_r = 0.7;
    p.wn_e = 257.04; % (rad/s)
    p.wn_a = 257.04; % (rad/s)
    p.wn_r = 257.04; % (rad/s)
    
    % Calcula los coeficientes aerodinámicos de los actuadores linealizando
    % en ángulo de ataque y deflexión de cada actuador.
    first = find(aero{1}.alpha == 2);
    last = find(aero{1}.alpha == 10);
    
    % Cálculos para elevador.
    p.CNde = (mean(aero{2}.cn(first:last,:) - aero{1}.cn(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{3}.cn(first:last,:) - aero{1}.cn(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.CNde = [p.CNde(1) p.CNde];

    p.Cmde = (mean(aero{2}.cm(first:last,:) - aero{1}.cm(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{3}.cm(first:last,:) - aero{1}.cm(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Cmde = [p.Cmde(1) p.Cmde];

    % Cálculos para timón de dirección.
    p.CYdr = (mean(aero{4}.cy(first:last,:) - aero{1}.cy(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{5}.cy(first:last,:) - aero{1}.cy(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.CYdr = [p.CYdr(1) p.CYdr];

    p.Cndr = (mean(aero{4}.cln(first:last,:) - aero{1}.cln(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{5}.cln(first:last,:) - aero{1}.cln(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Cndr = [p.Cndr(1) p.Cndr];

    % Cálculos para alerones.
    p.Clda = (mean(aero{6}.cll(first:last,:) - aero{1}.cll(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{7}.cll(first:last,:) - aero{1}.cll(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Clda = [p.Clda(1) p.Clda];
end