% Generar vectores X, Y, Z para una curva polinomial
n = 10; % Número de puntos de la curva
X = linspace(-5, 5, n);
Y = X.^2; % Ejemplo: parábola en Y
Z = X.^3; % Ejemplo: curva cúbica en Z

% Dibujar la curva 3D
figure;
plot3(X, Y, Z, 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Curva Polinomial 3D');


%% Pedir la cantidad de puntos
num_points = input('Ingrese el número de puntos para la interpolación: ');

% Pedir los puntos X, Y, Z
disp('Ingrese los puntos:');
X = zeros(1, num_points);
Y = zeros(1, num_points);
Z = zeros(1, num_points);
for i = 1:num_points
    fprintf('Punto %d:\n', i);
    X(i) = input('  X: ');
    Y(i) = input('  Y: ');
    Z(i) = input('  Z: ');
end

% Preguntar el tipo de interpolación
interp_type = input('Seleccione el tipo de interpolación (1 = lineal, 2 = polinomial): ');

if interp_type == 2
    degree = input('Ingrese el grado del polinomio: ');
else
    degree = 1; % Para interpolación lineal
end

% Número de puntos interpolados
interp_points = input('Ingrese el número de puntos que desea calcular: ');

% Interpolación
t = linspace(0, 1, num_points); % Parámetro t para los puntos originales
ti = linspace(0, 1, interp_points); % Parámetro t para los puntos interpolados

% Interpolación en cada dimensión
if interp_type == 2
    % Polinomial
    pX = polyfit(t, X, degree);
    pY = polyfit(t, Y, degree);
    pZ = polyfit(t, Z, degree);
    Xi = polyval(pX, ti);
    Yi = polyval(pY, ti);
    Zi = polyval(pZ, ti);
else
    % Lineal
    Xi = interp1(t, X, ti, 'linear');
    Yi = interp1(t, Y, ti, 'linear');
    Zi = interp1(t, Z, ti, 'linear');
end

% Dibujar la curva original y la interpolada
figure;
plot3(X, Y, Z, 'o-', 'LineWidth', 1.5, 'DisplayName', 'Puntos Originales');
hold on;
plot3(Xi, Yi, Zi, 'r-', 'LineWidth', 2, 'DisplayName', 'Interpolación');
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Interpolación de Curva 3D');
legend;
hold off;

% Guardar puntos interpolados
disp('Puntos calculados de la interpolación:');
interp_result = [Xi; Yi; Zi]';
disp(interp_result);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55

% Supongamos que los vectores son:

p_x = Xi; % Ejemplo de datos
p_y = Yi;
p_z = Zi;
phi = ones(1, 100)*3.14159;   % Ángulos aleatorios entre 0 y pi
theta = ones(1, 100)*-3.14159/2;
psi = ones(1, 100)*0;

% Matriz para almacenar los resultados de los ángulos
q_matrix = zeros(5, 100);

% Bucle para calcular los ángulos para cada conjunto de valores
for i = 1:100
    % Ejecutar la función con los valores correspondientes
    angulos = cinematica_inv(0.1920, 0.1820, 0.2020, 0.1238, 0.0950, ...
                              p_x(i), p_y(i), p_z(i), ...
                              phi(i), theta(i), psi(i));
    % Almacenar los resultados en la matriz
    q_matrix(i, :) = angulos;
end

% Visualizar la matriz de ángulos resultante
disp('Matriz de ángulos (q1, q2, q3, q4, q5):');
disp(q_matrix);
