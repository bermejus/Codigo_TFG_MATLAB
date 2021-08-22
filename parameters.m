function p = parameters()
    %% Load aerodynamic data from missile DATCOM
    aero = datcomimport("for006.dat");

    %% Build parameter list
    p.a = 340.3; % m/s
    p.rho = 1.225; % kg/m^3
    p.g = 9.81; % m/s^2

    p.l = 1.1; % m
    p.d = 0.127; % m
    p.Sref = pi*(p.d/2)^2; % m^2
    p.c = 0.05; % m
    p.b = 2*0.134; % m
    p.x0 = 0.446; % m (Point around which aerodynamic moments have been calculated) [CG at burnout]

    p.tb = 5.2; % s
    p.time_lut = [0.0, 0.3, 0.6, 1.2, 1.8, 2.4, 4.2, 5.2]; % s (lut == lookup table)
    p.thrust_lut = [0.0, 570.0, 650.0, 750.0, 770.0, 650.0, 50.0, 0.0]; % N
    p.mass_lut = [11.25, 11.16, 11.06, 10.82, 10.58, 10.38, 10.16, 10.15]; % kg
    p.cg_lut = [0.565, 0.555, 0.544, 0.519, 0.493, 0.471, 0.447, 0.446]; % m
    p.Ixx_lut = [0.0252, 0.0250, 0.0248, 0.0244, 0.0239, 0.0235, 0.0231, 0.0231]; % kg*m^2
    p.Iyy_lut = [0.985, 0.979, 0.973, 0.958, 0.942, 0.929, 0.915, 0.914]; % kg*m^2
    p.Ixx_dot_lut = [0.0, diff(p.Ixx_lut)./diff(p.time_lut)]; % kg*m^2/s
    p.Iyy_dot_lut = [0.0, diff(p.Iyy_lut)./diff(p.time_lut)]; % kg*m^2/s

    p.alpha_lut = deg2rad(aero{1}.alpha);
    p.mach_lut = [0.0 aero{1}.mach];
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

    p.vanes_cmd = [0.0, 0.0, 0.0];
    p.deltas_cmd = [0.0, 0.0, 0.0];

    p.vanes_e = [0.0; 0.0];
    p.vanes_r = [0.0; 0.0];
    p.deltas_e = [0.0; 0.0];
    p.deltas_a = [0.0; 0.0];
    p.deltas_r = [0.0; 0.0];

    p.damp_e = 0.7;
    p.damp_a = 0.7;
    p.damp_r = 0.7;
    p.wn_e = 257.04; % rad/s
    p.wn_a = 257.04; % rad/s
    p.wn_r = 257.04; % rad/s

    p.damp_te = 0.7;
    p.damp_tr = 0.7;
    p.wn_te = 257.04; % rad/s
    p.wn_tr = 257.04; % rad/s
    
    %% Actuator coefficients
    first = find(aero{1}.alpha == 2);
    last = find(aero{1}.alpha == 10);
    
    % Calculate linear aerodynamic coefficients for elevator deflection
    p.CNde = (mean(aero{2}.cn(first:last,:) - aero{1}.cn(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{3}.cn(first:last,:) - aero{1}.cn(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.CNde = [p.CNde(1) p.CNde];

    p.Cmde = (mean(aero{2}.cm(first:last,:) - aero{1}.cm(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{3}.cm(first:last,:) - aero{1}.cm(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Cmde = [p.Cmde(1) p.Cmde];

    % Calculate linear aerodynamic coefficients for rudder deflection
    p.CYdr = (mean(aero{4}.cy(first:last,:) - aero{1}.cy(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{5}.cy(first:last,:) - aero{1}.cy(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.CYdr = [p.CYdr(1) p.CYdr];

    p.Cndr = (mean(aero{4}.cln(first:last,:) - aero{1}.cln(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{5}.cln(first:last,:) - aero{1}.cln(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Cndr = [p.Cndr(1) p.Cndr];

    % Calculate linear aerodynamic coefficients for aileron deflection
    p.Clda = (mean(aero{6}.cll(first:last,:) - aero{1}.cll(first:last,:), 1)/deg2rad(5.0) + ...
              mean(aero{7}.cll(first:last,:) - aero{1}.cll(first:last,:), 1)/deg2rad(10.0)) / 2;
    p.Clda = [p.Clda(1) p.Clda];
end