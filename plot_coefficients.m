%% Datos aerodinámicos del misil Javelin
% Cargar los datos aerodinámicos obtenidos con el programa Missile DATCOM
% revisión 3/99 mediante la lectura de un fichero de texto (for006.dat).
aero = datcomimport("for006.dat");

%% Graficar coeficientes aerodinámicos
alpha = aero{1}.alpha;
mach = aero{1}.mach;

% Coeficientes aerodinámicos longitudinales
plot_coefficient(alpha, mach, aero{1}.cn, "C_{N} frente a \alpha", "C_{N}");
plot_coefficient(alpha, mach, aero{1}.cnq, "C_{Nq} frente a \alpha", "C_{Nq}");
plot_coefficient(alpha, mach, aero{1}.cm, "C_{m} frente a \alpha", "C_{m}");
plot_coefficient(alpha, mach, aero{1}.cma, "C_{m\alpha} frente a \alpha", "C_{m\alpha}");
plot_coefficient(alpha, mach, aero{1}.cmq, "C_{mq} frente a \alpha", "C_{mq}");

% Coeficientes aerodinámicos lateral-direccionales
plot_coefficient(alpha, mach, aero{1}.clb, "C_{l\beta} frente a \alpha", "C_{l\beta}");
plot_coefficient(alpha, mach, aero{1}.clp, "C_{lp} frente a \alpha", "C_{lp}");
plot_coefficient(alpha, mach, aero{1}.clr, "C_{lr} frente a \alpha", "C_{lr}");

plot_coefficient(alpha, mach, aero{1}.cnb, "C_{n\beta} frente a \alpha", "C_{n\beta}");
plot_coefficient(alpha, mach, aero{1}.cnp, "C_{np} frente a \alpha", "C_{np}");
plot_coefficient(alpha, mach, aero{1}.cnr, "C_{nr} frente a \alpha", "C_{nr}");

% Efectos que produce una deflexión de elevador determinada
plot_coefficient(alpha, mach, aero{2}.cn - aero{1}.cn, "{\Delta}C_{N} frente a \alpha ({\delta}_{e}=5\circ)", "{\Delta}C_{N}");
plot_coefficient(alpha, mach, aero{3}.cn - aero{1}.cn, "{\Delta}C_{N} frente a \alpha ({\delta}_{e}=10\circ)", "{\Delta}C_{N}");
plot_coefficient(alpha, mach, aero{2}.cm - aero{1}.cm, "{\Delta}C_{m} frente a \alpha ({\delta}_{e}=5\circ)", "{\Delta}C_{m}");
plot_coefficient(alpha, mach, aero{3}.cm - aero{1}.cm, "{\Delta}C_{m} frente a \alpha ({\delta}_{e}=10\circ)", "{\Delta}C_{m}");

% Efectos que produce una deflexión de timón de dirección (rudder) determinada
plot_coefficient(alpha, mach, aero{4}.cy - aero{1}.cy, "{\Delta}C_{Y} frente a \alpha ({\delta}_{r}=5\circ)", "{\Delta}C_{Y}");
plot_coefficient(alpha, mach, aero{5}.cy - aero{1}.cy, "{\Delta}C_{Y} frente a \alpha ({\delta}_{r}=10\circ)", "{\Delta}C_{Y}");
plot_coefficient(alpha, mach, aero{4}.cln - aero{1}.cln, "{\Delta}C_{n} frente a \alpha ({\delta}_{r}=5\circ)", "{\Delta}C_{n}");
plot_coefficient(alpha, mach, aero{5}.cln - aero{1}.cln, "{\Delta}C_{n} frente a \alpha ({\delta}_{r}=10\circ)", "{\Delta}C_{n}");

% Efectos que produce una deflexión de los alerones determinada
plot_coefficient(alpha, mach, aero{6}.cll - aero{1}.cll, "{\Delta}C_{l} frente a \alpha ({\delta}_{a}=5\circ)", "{\Delta}C_{l}");
plot_coefficient(alpha, mach, aero{7}.cll - aero{1}.cll, "{\Delta}C_{l} frente a \alpha ({\delta}_{a}=10\circ)", "{\Delta}C_{l}");

%% Gráfica de la curva de empuje
time = [0.0, 0.3, 0.6, 1.2, 1.8, 2.4, 4.2, 5.2]; % s
thrust = [0.0, 570.0, 650.0, 750.0, 770.0, 650.0, 50.0, 0.0]; % N

figure;
plot(time, thrust);
title("Curva de empuje");
xlabel("Tiempo (s)");
ylabel("Empuje (N)");
xlim([time(1), time(end)]);
grid();

%% Función que realiza una interpolación de los coeficientes aerodinámicos
%  y los prepara para ser representados en una gráfica. Se muestra una
%  curva para cada número de Mach considerado.
function plot_coefficient(alpha, mach, coeff, title_str, ylabel_str)
    smoothed_alpha = linspace(alpha(1), alpha(end), 100);
    
    figure;
    plot(smoothed_alpha, interp2(mach, alpha, coeff, mach(1), smoothed_alpha, "makima"));
    if length(mach) > 1
        hold on;
        for i=2:length(mach)
            plot(smoothed_alpha, interp2(mach, alpha, coeff, mach(i), smoothed_alpha, "makima"));
        end
        hold off;
    end
    title(title_str);
    xlabel("\alpha (\circ)");
    ylabel(ylabel_str);
    grid();
    
    mach_str = [];
    for i=1:length(mach)
        mach_str = [mach_str, "M=" + string(mach(i))];
    end
    
    legend(mach_str);
end