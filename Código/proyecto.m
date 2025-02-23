%% DISEÑO E IMPLEMENTACIÓN DE UN ROBOT DE 5GDL CON PROPÓSITO PICK AND PLACE

clc, clear, close

%% ::::::::::::::::::::::: CINEMÁTICA ::::::::::::::::::::::::::
%% CINEMÁTICA DIRECTA

% Parametros Denavid-Hartenberg
syms q1 q2 q3 q4 q5 L1 L2 L3 L4 L5 pi

theta = [q1; q2; q3; q4+pi/2; q5];
d = [L1; 0; 0; 0; L5];
a = [0; L2; L3; L4; 0];
alpha = [pi/2; 0; 0; pi/2; 0];
MTH = [theta d a alpha];

T_01 = denavit(theta(1), d(1), a(1), alpha(1));
T_12 = denavit(theta(2), d(2), a(2), alpha(2));
T_23 = denavit(theta(3), d(3), a(3), alpha(3));
T_34 = denavit(theta(4), d(4), a(4), alpha(4));
T_45 = denavit(theta(5), d(5), a(5), alpha(5));

% MTH
T_05 = T_01*T_12*T_23*T_34*T_45;
fprintf(":::::::::::::. CINEMÁTICA ::::::::::::::::::::::\n")
fprintf("Matriz de transformación homogénena:\n\n")
disp(T_05)

% Posicion
P_05 = T_05(1 : 3, 4);
fprintf("Posición del sistema 5:\n\n")
disp(P_05)

%% CINEMÁTICA DIFERENCIAL
syms dq1 dq2 dq3 dq4 dq5

T_02 = T_01*T_12;
T_03 = T_02*T_23;
T_04 = T_03*T_34;

% Velocidad angular
z0=[0;0;1];
z1=T_01(1:3,1:3)*z0;
z2=T_02(1:3,1:3)*z0;
z3=T_03(1:3,1:3)*z0;
z4=T_04(1:3,1:3)*z0;

e1=1; % Revoluta
e2=1; % Revoluta
e3=1; % Revoluta
e4=1; % Revoluta
e5=1; % Revoluta

Jw=[e1*z0, e2*z1, e3*z2, e4*z3, e5*z4];
dq=[dq1; dq2; dq3; dq4; dq5];
W05=Jw*dq;

W01=dq1*e1*z0;
W02=W01+dq2*e2*z1;
W03=W02+dq3*e3*z2;
W04=W03+dq4*e4*z3;

fprintf("VELOCIDADES ANGULARES:\n")
fprintf("Sistema 1 con respecto a 0:\n\n")
disp(W01)
fprintf("Sistema 2 con respecto a 0:\n\n")
disp(W02)
fprintf("Sistema 3 con respecto a 0:\n\n")
disp(W03)
fprintf("Sistema 4 con respecto a 0:\n\n")
disp(W04)
fprintf("Sistema 5 con respecto a 0:\n\n")
disp(W05)
fprintf("JACOBIANO DE VELOCIDAD ANGULAR:\n")
disp(Jw)

% Velocidad lineal

P_01 = T_01(1 : 3, 4);
P_02 = T_02(1 : 3, 4);
P_03 = T_03(1 : 3, 4);
P_04 = T_04(1 : 3, 4);

V01 = diff(P_01,q1)*dq1;
V02 = diff(P_02,q1)*dq1+diff(P_02,q2)*dq2;
V03 = diff(P_03,q1)*dq1+diff(P_03,q2)*dq2+diff(P_03,q3)*dq3;
V04 = diff(P_04,q1)*dq1+diff(P_04,q2)*dq2+diff(P_04,q3)*dq3+...
diff(P_04,q4)*dq4;
V05 = diff(P_05,q1)*dq1+diff(P_05,q2)*dq2+diff(P_05,q3)*dq3+...
    diff(P_05,q4)*dq4+diff(P_05,q5)*dq5;

Jv = [diff(P_05,q1),diff(P_05,q2),diff(P_05,q3),diff(P_05,q4),...
    diff(P_05,q5)];
J = [Jv;Jw];

fprintf("VELOCIDADES LINEALES:\n")
fprintf("Sistema 1 con respecto a 0:\n\n")
disp(V01)
fprintf("Sistema 2 con respecto a 0:\n\n")
disp(V02)
fprintf("Sistema 3 con respecto a 0:\n\n")
disp(V03)
fprintf("Sistema 4 con respecto a 0:\n\n")
disp(V04)
fprintf("Sistema 5 con respecto a 0:\n\n")
disp(V05)
fprintf("JACOBIANO DE VELOCIDAD LINEAL:\n")
disp(Jv)
fprintf("MATRIZ JACOBIANA:\n")
disp(J)

%% ::::::::::::::::::::::: ESTÁTICA :::::::::::::::::::::::
%% Datos para el cálculo estático
syms Fx Fy Fz Mx My Mz g
syms m1_eje % masa del eje de 8mm junto con rodamiento y acople
syms m2 m3 m4 % masa de las platinas
syms m_servo_efector % masa del servo que hace rotar el efector final
syms m_efector % masa del efector final con el servo de abajo, no el que lo hace rotar
syms m_base % masa platina base
syms m_servo % masa servomotor

% Parámetros de longitud real de eslabones
% L1 L2 L3 L4 L5 son la suma de todos los componentes 
% en un solo eslabón (longitud total)
syms Lc2 Lc3 Lc4 % platinas
syms L1_eje % eje 8 mm con 6.35
syms L_base % Base (platina que va con el servo 60 kg - espesor)
syms L_servo_largo % Bracket fijo servo hasta actuador
syms L_servo_bracket % Bracket giratorio ancho hasta actuador (se ignoró su peso)
syms L_efector_base % Base hasta el actuador del servo del efector
syms L_efector_final 

F56 = [Fx; Fy; Fz]; % Fuerza aplicada en el efector final

M56 = [Mx; My; Mz]; % Par aplicado en el efector final
G = [0;0;-g]; % Con respecto a la base

R_45 = T_45(1:3,1:3);
R_34 = T_34(1:3,1:3);
R_23 = T_23(1:3,1:3);
R_12 = T_12(1:3,1:3);
R_01 = T_01(1:3,1:3);
R_02 = T_02(1:3,1:3);
R_03 = T_03(1:3,1:3);
R_04 = T_04(1:3,1:3);
R_05 = T_05(1:3,1:3);

R_50 = R_05.';
R_40 = R_04.';
R_30 = R_03.';
R_20 = R_02.';
R_10 = R_01.';

% Posiciones
% ri_(i-1)i: Posición de i-1 a i con respecto a i
r5_45 = [0;0;L5];
r5_4c_1 = [L_efector_final/2;0;0]; % actuador del servo del efector hasta efector final

r4_34 = [0;0;L4];
r4_3c_1 = [L_servo_bracket+L_base/2;0;0]; % base 
r4_3c_2 = [L_servo_bracket+L_base+Lc4/2;0;0]; % platina
r4_3c_3 = [L_servo_bracket+L_base+Lc4+L_base/2;0;0]; % base
r4_3c_4 = [L_servo_bracket+L_base+Lc4+L_base+L_efector_base/2;0;0]; % Base hasta el actuador del servo del efector

r3_23 = [L3;0;0];
r3_2c_1 = [L_servo_bracket+L_base/2;0;0]; % base
r3_2c_2 = [L_servo_bracket+L_base+Lc3/2;0;0]; % platina
r3_2c_3 = [L_servo_bracket+L_base+Lc3+L_base/2;0;0]; % base
r3_2c_4 = [L_servo_bracket+L_base+Lc3+L_base+L_servo_largo/2;0;0]; % Bracket fijo servo hasta actuador

r2_12 = [L2;0;0];
r2_1c_1 = [L_servo_bracket+L_base/2;0;0]; % base
r2_1c_2 = [L_servo_bracket+L_base+Lc2/2;0;0]; % platina
r2_1c_3 = [L_servo_bracket+L_base+Lc2+L_base/2;0;0]; % base
r2_1c_4 = [L_servo_bracket+L_base+Lc2+L_base+L_servo_largo/2;0;0]; % Bracket fijo servo hasta actuador

r1_01 = [0;L1;0];
r1_0c_1 = [0;L1_eje/2;0]; % eje 8 mm con 6.35
r1_0c_2 = [0;L1_eje+L_base/2;0]; % base
r1_0c_3 = [0;L1_eje+L_base+L_servo_largo/2;0]; % Bracket fijo servo hasta actuador

% Equilibrio de fuerzas
F45 = simplify(R_45*F56 - m_efector*R_40*G);
F34 = simplify(R_34*F45 - 2*m_base*R_30*G - m4*R_30*G - m_servo_efector*R_30*G);
F23 = simplify(R_23*F34 - 2*m_base*R_20*G - m3*R_20*G - m_servo*R_20*G);
F12 = simplify(R_12*F23 - 2*m_base*R_10*G - m2*R_10*G - m_servo*R_10*G);
F01 = simplify(R_01*F12 - m1_eje*eye(3)*G - m_base*eye(3)*G - m_servo*eye(3)*G);

% Equilibrio en Momentos
M45 = R_45*M56 + cross(R_45*r5_45, R_45*F56) - cross(R_45*r5_4c_1, m_efector*R_40*G);
M34 = R_34*M45 + cross(R_34*r4_34, R_34*F45) - cross(R_34*r4_3c_1, m_base*R_30*G) - cross(R_34*r4_3c_2, m4*R_30*G) - cross(R_34*r4_3c_3, m_base*R_30*G) - cross(R_34*r4_3c_4, m_servo_efector*R_30*G);
M23 = R_23*M34 + cross(R_23*r3_23, R_23*F34) - cross(R_23*r3_2c_1, m_base*R_20*G) - cross(R_23*r3_2c_2, m3*R_20*G) - cross(R_23*r3_2c_3, m_base*R_20*G) - cross(R_23*r3_2c_4, m_servo *R_20*G);
M12 = R_12*M23 + cross(R_12*r2_12, R_12*F23) - cross(R_12*r2_1c_1, m_base*R_10*G) - cross(R_12*r2_1c_2, m2*R_10*G) - cross(R_12*r2_1c_3, m_base*R_10*G) - cross(R_12*r2_1c_4, m_servo *R_10*G);
M01 = R_01*M12 + cross(R_01*r1_01, R_01*F12) - cross(R_01*r1_0c_1, m1_eje*eye(3)*G) - cross(R_01*r1_0c_2, m_base*eye(3)*G) - cross(R_01*r1_0c_3, m_servo*eye(3)*G); % esos 3 productos dan 0

% Método Trabajo Virtual (considera los momentos 0 y la gravedad 0)
Tau = simplify(Jv.'*R_05*F56);
fprintf("VECTOR DE MOMENTOS SIMBÓLICA:\n")
disp(Tau)

%% Reemplazo numérico para el cálculo estático
e_n = 3e-03; % Espesor de la platina de acero: 3 mm
ancho_n = 32e-03; % Ancho de la platina de acero: 32 mm
largo_n = 67e-03; % Largo de la platina de acero (base): 67 mm

% LONGITUD DE PLATINAS 
Platina2 = 0.07; % m -> 7cm
Platina3 = 0.09; % m -> 9cm
Platina4 = 0.04; % m -> 4cm

Lc2_n = Platina2/2; % m
Lc3_n = Platina3/2; % m
Lc4_n = Platina4/2; % m

L_servo_largo_n = 72e-03; % m
L1_eje_PaP_n = 20e-03; % m
L1_eje_8mm_n = 97e-03; % mm cortarlo a 100 mm
L1_eje_n = L1_eje_PaP_n+L1_eje_8mm_n;
L_base_n = e_n; % m
L_servo_bracket_n = 34e-03; % m

L_efector_base_n = 43.8e-03; % m
L_efector_final_n = 95e-03; % m

V2_n = Platina2*ancho_n*e_n; % m^3
V3_n = Platina3*ancho_n*e_n; % m^3
V4_n = Platina4*ancho_n*e_n; % m^3
Vbase_n = largo_n*ancho_n*e_n; % m^3

% Densidad de la platina de acero
rho_n = 7604.16667; % kg/m^3

m1_eje_n = 83e-03; % kg
m1_eje_PaP_n = 17e-03; % kg
m2_n = 2*rho_n*V2_n; % kg
m3_n = 2*rho_n*V3_n; % kg
m4_n = 2*rho_n*V4_n; % kg
m_efector_n = 122e-03;% kg
m_servo_efector_n = 55e-03; % kg
m_base_n = rho_n*Vbase_n; % kg
m_servo_n = 223e-03; % kg

g_n = 9.81; % m/s^2

Fx_n = -0.7*g_n; % N
Fy_n = 0; % N
Fz_n = 0; % N
Mx_n = 0; % N.m
My_n = 0; % N.m
Mz_n = 0; % N.m

q1_n = 0; % rad
q2_n = 0; % rad
q3_n = 0; % rad
q4_n = 0; % rad
q5_n = 0; % rad

pi_n = 3.141592; % rad

L1_n = L1_eje_n+L_base_n+L_servo_largo_n;
L2_n = L_servo_bracket_n+L_base_n+Platina2+L_base_n+L_servo_largo_n;
L3_n = L_servo_bracket_n+L_base_n+Platina3+L_base_n+L_servo_largo_n;
L4_n = L_servo_bracket_n+L_base_n+Platina4+L_base_n+L_efector_base_n;
L5_n = L_efector_final_n;

% Momentos hallados mediante el método de trabajo virtual

Tau_n  = subs(Tau,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L_efector_final_n]);

Tau_n  = subs(Tau_n,[L1 L2 L3 L4 L5],...
    [L1_n L2_n L3_n L4_n L5_n]);

Tau_n  = subs(Tau_n,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

Tau_n  = subs(Tau_n,[Fx Fy Fz Mx My Mz],...
    [Fx_n Fy_n Fz_n Mx_n My_n Mz_n]);

Tau_n  = subs(Tau_n,[q1 q2 q3 q4 q5],...
    [q1_n q2_n q3_n q4_n q5_n]);

Tau_n  = subs(Tau_n,[pi g],[pi_n g_n]); % N.m
Tau_n_kgcm=double(Tau_n.*10.2); % kg.cm
fprintf(":::::::::::::. ESTÁTICA ::::::::::::::::::::::\n")
fprintf("Vector de momentos hallados mediante el método de trabajo virtual (kg.cm)->:\n")
disp(Tau_n_kgcm)

% Momentos hallados individualmente
M01_n  = subs(M01,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 L_efector_base...
    m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo Fx Fy Fz Mx My Mz q1 q2 q3 q4 q5 pi g L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n...
    L4_n L5_n L_efector_base_n m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n Fx_n ...
    Fy_n Fz_n Mx_n My_n Mz_n q1_n q2_n q3_n q4_n q5_n pi_n g_n L_efector_final_n]);
M01_n_z = double(M01_n'*[0; 0; 1]); % N.m
fprintf("Momentos hallados individualmente->:\n")
fprintf("Par 1 (N.m):\n")
disp(M01_n_z)

M12_n  = subs(M12,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 L_efector_base...
    m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo Fx Fy Fz Mx My Mz q1 q2 q3 q4 q5 pi g L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n...
    L4_n L5_n L_efector_base_n m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n Fx_n ...
    Fy_n Fz_n Mx_n My_n Mz_n q1_n q2_n q3_n q4_n q5_n pi_n g_n L_efector_final_n]);
M12_n_z = double((M12_n)'*[0; 0; 1])*10.2; % N.m
fprintf("Par 2 (kg.cm):\n")
disp(M12_n_z)

M23_n  = subs(M23,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 L_efector_base...
    m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo Fx Fy Fz Mx My Mz q1 q2 q3 q4 q5 pi g L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n...
    L4_n L5_n L_efector_base_n m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n Fx_n ...
    Fy_n Fz_n Mx_n My_n Mz_n q1_n q2_n q3_n q4_n q5_n pi_n g_n L_efector_final_n]);
M23_n_z = double((M23_n)'*[0; 0; 1])*10.2;
fprintf("Par 3 (kg.cm):\n")
disp(M23_n_z)

M34_n  = subs(M34,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 L_efector_base...
    m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo Fx Fy Fz Mx My Mz q1 q2 q3 q4 q5 pi g L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n...
    L4_n L5_n L_efector_base_n m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n Fx_n ...
    Fy_n Fz_n Mx_n My_n Mz_n q1_n q2_n q3_n q4_n q5_n pi_n g_n L_efector_final_n]);
M34_n_z = double((M34_n)'*[0; 0; 1])*10.2;
fprintf("Par 4 (kg.cm):\n")
disp(M34_n_z)

M45_n  = subs(M45,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 L_efector_base...
    m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo Fx Fy Fz Mx My Mz q1 q2 q3 q4 q5 pi g L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n...
    L4_n L5_n L_efector_base_n m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n Fx_n ...
    Fy_n Fz_n Mx_n My_n Mz_n q1_n q2_n q3_n q4_n q5_n pi_n g_n L_efector_final_n]);
M45_n_z = double((M45_n)'*[0; 0; 1])*10.2;
fprintf("Par 5 (kg.cm):\n")
disp(M45_n_z)

% ##################### DINÁMICA #######################################
%% ALGORITMO COMPUTACIONAL DE NEWTON-EULER PARA EL MODELADO DINÁMICO DEL ROBOT
fprintf(":::::::::::::. DINÁMICA ::::::::::::::::::::::\n")
% PASO 1 -> Establecer los sistemas de acuerdo a DH:
% PASO 2 -> Condiciones iniciales del sistema 0:
w0_0 = [sym(0);sym(0);sym(0)];
dw0_0 = [sym(0);sym(0);sym(0)];
v0_0 = [sym(0);sym(0);sym(0)];
dv0_0 = [sym(0);sym(0);-g];

fprintf("CONDICIONES INICIALES:\n")
fprintf("Velocidad angular del sistema base:\n")
disp(w0_0)
fprintf("Aceleración angular del sistema base:\n")
disp(dw0_0)
fprintf("Velocidad lineal del sistema base:\n")
disp(v0_0)
fprintf("Aceleración lineal del sistema base:\n")
disp(dv0_0)

% Fuerza y par ejercidos en el extremo del robot (sistema 5)
f6_6=[-0.7*g_n;sym(0);sym(0)];
t6_6=[sym(0);sym(0);sym(0)];

fprintf("Fuerza en el extremo del robot:\n")
disp(f6_6)
fprintf("Par en el extremo del robot:\n")
disp(t6_6)

% Vectores posición que unen el origen {S_i-1} con {S_i} con respecto 
% al sistema i 

fprintf("Vector posición de 0 hacia 1 con respecto a 1:\n")
disp(r1_01)
fprintf("Vector posición de 1 hacia 2 con respecto a 2:\n")
disp(r2_12)
fprintf("Vector posición de 2 hacia 3 con respecto a 3:\n")
disp(r3_23)
fprintf("Vector posición de 3 hacia 4 con respecto a 4:\n")
disp(r4_34)
fprintf("Vector posición de 4 hacia 5 con respecto a 5:\n")
disp(r5_45)

% Coordenadas del centro de masas de los eslabones i con respecto al
% sistema {S_i}

S1_1 = r1_0c_2; 
S2_2 = r2_1c_2;
S3_3 = r3_2c_2;
S4_4 = r4_3c_2;
S5_5 = r5_4c_1;
S6_6 = [sym(0);sym(0);sym(0)];

syms I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz

I1_1 = [I1xx, 0, 0; 0, I1yy, 0; 0, 0, I1zz];
I2_2 = [I2xx, 0, 0; 0, I2yy, 0; 0, 0, I2zz];
I3_3 = [I3xx, 0, 0; 0, I3yy, 0; 0, 0, I3zz];
I4_4 = [I4xx, 0, 0; 0, I4yy, 0; 0, 0, I4zz];
I5_5 = [I5xx, 0, 0; 0, I5yy, 0; 0, 0, I5zz];

% PASO 3 -> Matrices de rotación y sus inversas (halladas anteriormente)

R_54=R_45.';
R_43=R_34.';
R_32=R_23.';
R_21=R_12.';

R_56 = eye(3);
R_65 = R_56.';

% PASO 4 -> Velocidades angulares con respecto al sistema {S_i}
w1_1 = R_10*(w0_0+z0*dq1);
w2_2 = R_21*(w1_1+z1*dq2);
w3_3 = R_32*(w2_2+z2*dq3);
w4_4 = R_43*(w3_3+z3*dq4);
w5_5 = R_54*(w4_4+z4*dq5);

% PASO 5 -> Aceleraciones angulares con respecto al sistema {S_i}
syms ddq1 ddq2 ddq3 ddq4 ddq5

dw1_1 = simplify(R_10*(dw0_0+z0*ddq1) + cross(w0_0,z0*dq1));
dw2_2 = simplify(R_21*(dw1_1+z0*ddq2) + cross(w1_1,z0*dq2));
dw3_3 = simplify(R_32*(dw2_2+z0*ddq3) + cross(w2_2,z0*dq3));
dw4_4 = simplify(R_43*(dw3_3+z0*ddq4) + cross(w3_3,z0*dq4));
dw5_5 = simplify(R_54*(dw4_4+z0*ddq5) + cross(w4_4,z0*dq5));

% PASO 6 -> Aceleraciones lineales del sistema {S_i}
dv1_1 = cross(dw1_1,r1_01)+cross(w1_1,cross(w1_1,r1_01))+R_10*dv0_0;
dv2_2 = cross(dw2_2,r2_12)+cross(w2_2,cross(w2_2,r2_12))+R_21*dv1_1;
dv3_3 = cross(dw3_3,r3_23)+cross(w3_3,cross(w3_3,r3_23))+R_32*dv2_2;
dv4_4 = cross(dw4_4,r4_34)+cross(w4_4,cross(w4_4,r4_34))+R_43*dv3_3;
dv5_5 = cross(dw5_5,r5_45)+cross(w5_5,cross(w5_5,r5_45))+R_54*dv4_4;

% PASO 7 -> Aceleración lineal del centro de gravedad del eslabon i
dvs1_1 = cross(dw1_1,S1_1)+cross(w1_1,cross(w1_1,r1_01))+dv1_1;
dvs2_2 = cross(dw2_2,S2_2)+cross(w2_2,cross(w2_2,r2_12))+dv2_2;
dvs3_3 = cross(dw3_3,S3_3)+cross(w3_3,cross(w3_3,r3_23))+dv3_3;
dvs4_4 = cross(dw4_4,S4_4)+cross(w4_4,cross(w4_4,r4_34))+dv4_4;
dvs5_5 = cross(dw5_5,S5_5)+cross(w5_5,cross(w5_5,r5_45))+dv5_5;

% PASO 8 -> Fuerza ejercida sobre el eslabon i
f5_5 = R_56*f6_6+(m_efector+m_servo_efector)*dvs5_5;
f4_4 = R_45*f5_5+(2*m_base+m4)*dvs4_4;
f3_3 = R_34*f4_4+(2*m_base+m3)*dvs3_3;
f2_2 = R_23*f3_3+(2*m_base+m2)*dvs2_2;
f1_1 = R_12*f2_2+(m_base+m1_eje)*dvs1_1;

% PASO 9 -> Par ejercido sobre el eslabón i
t5_5 = R_56*(t6_6+cross(R_65*r5_45,f6_6))+cross(r5_45+S5_5,(m_efector+m_servo_efector)*dvs5_5)+...
    I5_5*w5_5+cross(w5_5,I5_5*w5_5);
t4_4 = R_45*(t5_5+cross(R_54*r4_34,f5_5))+cross(r4_34+S4_4,(2*m_base+m4)*dvs4_4)+...
    I4_4*w4_4+cross(w4_4,I4_4*w4_4);
t3_3 = R_34*(t4_4+cross(R_43*r3_23,f4_4))+cross(r3_23+S3_3,(2*m_base+m3)*dvs3_3)+...
    I3_3*w3_3+cross(w3_3,I3_3*w3_3);
t2_2 = R_23*(t3_3+cross(R_32*r2_12,f3_3))+cross(r2_12+S2_2,(2*m_base+m2)*dvs2_2)+...
    I2_2*w2_2+cross(w2_2,I2_2*w2_2);
t1_1 = R_12*(t2_2+cross(R_21*r1_01,f2_2))+cross(r1_01+S1_1,(m_base+m1_eje)*dvs1_1)+...
    I1_1*w1_1+cross(w1_1,I1_1*w1_1);

% PASO 10 -> Obtener la fuerza o par aplicado al eslabón i
% (traslación-fuerza, rotación-par)

t5 = t5_5.'*R_54*z0;
t4 = t4_4.'*R_43*z0;
t3 = t3_3.'*R_32*z0;
t2 = t2_2.'*R_21*z0;
t1 = t1_1.'*R_10*z0;

%% Reemplazo numérico para el cálculo dinámico de Newton-Euler

% velocidad articular
dq1_n = 1; % rad/s
dq2_n = 10; % rad/s
dq3_n = 0.4; % rad/s
dq4_n = 0.2; % rad/s
dq5_n = 0; % rad/s

% aceleración articular
ddq1_n = 0.1; % rad/s^2
ddq2_n = 0.1; % rad/s^2
ddq3_n = 0.01; % rad/s^2
ddq4_n = 0.005; % rad/s^2
ddq5_n = 0; % rad/s^2

% momentos de inercia
I1xx_n = 1/12*(m1_eje_PaP_n)*(L1_eje_PaP_n)^2 + m1_eje_PaP_n*(L1_eje_PaP_n+L1_eje_8mm_n)^2 +...
    1/12*(m1_eje_n)*(L1_eje_8mm_n)^2 + m1_eje_n*(L1_eje_8mm_n/2)^2;
I1yy_n = 1/2*(m1_eje_PaP_n)*(5e-3/2)^2 + 1/2*(m1_eje_n)*(8e-3/2)^2;
I1zz_n = I1xx_n;

I2xx_n = 1/12*(2*m_base_n*(largo_n^2+ancho_n^2) + 2*m2_n/2*(ancho_n^2+e_n^2)) + 2*m2_n/2*((largo_n+e_n)/2)^2;
I2yy_n = 1/12*(m_base_n)*(e_n^2+largo_n^2) + m_base_n*(L_servo_largo_n+e_n+Platina2+e_n/2)^2+...
    2*(1/12*(m2_n/2)*(e_n^2+Platina2^2) + (m2_n/2)*((largo_n+e_n)/2)^2) + ...
    1/12*(m_base_n)*(e_n^2+largo_n^2) + m_base_n*(L_servo_largo_n+e_n/2)^2;  
I2zz_n = 1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_servo_largo_n+e_n+Platina2+e_n/2)^2 + ...
    2*(1/12*m2_n/2)*(Platina2^2+ancho_n^2) + 2*(m2_n/2)*(L_servo_largo_n+e_n+Platina2/2)^2 + ...
    1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_servo_largo_n+e_n/2)^2;

I3xx_n = 1/12*(2*m_base_n*(largo_n^2+ancho_n^2) + 2*m2_n/2*(ancho_n^2+e_n^2)) + 2*m2_n/2*((largo_n+e_n)/2)^2; 
I3yy_n = 1/12*(m_base_n)*(e_n^2+largo_n^2) + m_base_n*(L_servo_largo_n+e_n+Platina3+e_n/2)^2+...
    2*(1/12*(m2_n/2)*(e_n^2+Platina3^2) + (m2_n/2)*((largo_n+e_n)/2)^2) + ...
    1/12*(m_base_n)*(e_n^2+largo_n^2) + m_base_n*(L_servo_largo_n+e_n/2)^2; 
I3zz_n = 1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_servo_largo_n+e_n+Platina3+e_n/2)^2 + ...
    2*(1/12*m2_n/2)*(Platina3^2+ancho_n^2) + 2*(m2_n/2)*(L_servo_largo_n+e_n+Platina3/2)^2 + ...
    1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_servo_largo_n+e_n/2)^2;

I4xx_n = 1/12*(2*(m_base_n)*(largo_n^2+e_n^2) + 2*(m4_n/2)*(Platina4^2+e_n^2)) + ...
    m_base_n*(L_efector_base_n+e_n+Platina4+e_n/2)^2 + 2*(m4_n/2)*(L_efector_base_n+e_n+Platina4/2)^2 ...
    + m_base_n*(L_efector_base_n+e_n/2)^2; 
I4yy_n = 1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_efector_base_n+e_n+Platina4+e_n/2)^2 + ...
    2*(1/12*(m4_n/2)*(ancho_n^2+Platina4^2)) + 2*(m4_n/2)*(L_efector_base_n+e_n+Platina4/2)^2 + ...
    1/12*(m_base_n)*(e_n^2+ancho_n^2) + m_base_n*(L_efector_base_n+e_n/2)^2; 
I4zz_n = 1/12*(m_base_n)*(ancho_n^2+largo_n^2) + m_base_n*(L_efector_base_n+e_n+Platina4+e_n/2)^2 + ...
    2*(1/12*(m4_n/2)*(e_n^2+ancho_n^2) + m4_n/2*((L_efector_base_n+e_n+Platina4/2)^2)) + ...
    1/12*(m_base_n)*(ancho_n^2+largo_n^2) + m_base_n*(L_efector_base_n+e_n/2)^2;

I5xx_n = 0.001; 
I5yy_n = 0.001; 
I5zz_n = 0.001;

%% Par 5
% Reemplazo de variables articulares
t5_n  = subs(t5,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

% Reemplazo de masas
t5_n  = subs(t5_n,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
t5_n  = subs(t5_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
t5_n  = subs(t5_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
t5_n  = subs(t5_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

%% Par 4
% Reemplazo de masas
t4_n  = subs(t4,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
t4_n  = subs(t4_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
t4_n  = subs(t4_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
t4_n  = subs(t4_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

t4_n_dq  = subs(t4_n,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq5],... % reemplaza dq2 menos ddq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq5_n]);

t4_n_ddq  = subs(t4_n,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],... % reemplaza ddq2 menos dq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

dq4_n_graf=-20:0.1:20;
ddq4_n_graf=-8:0.1:8;

t4_n_valor = subs(t4_n, [q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5], ...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]); % reemplaza dq2 y ddq2

t4_n_graf_vel = subs(t4_n_ddq, dq4, dq4_n_graf); 
figure
plot(dq4_n_graf,t4_n_graf_vel*10.2,'LineWidth',2.5,'Color','#1f41de')
hold on

t4_n_graf_ac = subs(t4_n_dq, ddq4, ddq4_n_graf); 
plot(ddq4_n_graf,t4_n_graf_ac*10.2,'LineWidth',2.5,'Color','#001780')

%% Par 3
% Reemplazo de masas
t3_n  = subs(t3,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
t3_n  = subs(t3_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
t3_n  = subs(t3_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
t3_n  = subs(t3_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

% Reemplazo de variables articulares
t3_n_dq  = subs(t3_n,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq4 ddq5],... % reemplaza dq2 menos ddq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq4_n ddq5_n]);

t3_n_ddq  = subs(t3_n,[q1 q2 q3 q4 q5 dq1 dq2 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],... % reemplaza ddq2 menos dq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

dq3_n_graf=dq4_n_graf;
ddq3_n_graf=ddq4_n_graf;

t3_n_valor = subs(t3_n, [q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5], ...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]); % reemplaza dq2 y ddq2

t3_n_graf_vel = subs(t3_n_ddq, dq3, dq3_n_graf); 
plot(dq3_n_graf,t3_n_graf_vel*10.2,'LineWidth',2.5,'Color','#40fc43')

t3_n_graf_ac = subs(t3_n_dq, ddq3, ddq3_n_graf); 
plot(ddq3_n_graf,t3_n_graf_ac*10.2,'LineWidth',2.5,'Color','#288429')

%% Par 2
% Reemplazo de masas
t2_n  = subs(t2,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
t2_n  = subs(t2_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
t2_n  = subs(t2_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
t2_n  = subs(t2_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

% Reemplazo de variables articulares
t2_n_dq  = subs(t2_n,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq3 ddq4 ddq5],... % reemplaza dq2 menos ddq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq3_n ddq4_n ddq5_n]);

t2_n_ddq  = subs(t2_n,[q1 q2 q3 q4 q5 dq1 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],... % reemplaza ddq2 menos dq2
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

dq2_n_graf=dq3_n_graf;
ddq2_n_graf=ddq3_n_graf;

t2_n_valor = subs(t2_n, [q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5], ...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]); % reemplaza dq2 y ddq2

t2_n_graf_vel = subs(t2_n_ddq, dq2, dq2_n_graf); 

plot(dq2_n_graf,t2_n_graf_vel*10.2,'LineWidth',2.5,'Color','#ff6c6c')

t2_n_graf_ac = subs(t2_n_dq, ddq2, ddq2_n_graf); 
plot(ddq2_n_graf,t2_n_graf_ac*10.2,'LineWidth',2.5,'Color','#9b0000')
legend('Variación de dq4', 'Variación de ddq4', 'Variación de dq3', 'Variación de ddq3', 'Variación de dq2', 'Variación de ddq2')
title('VARIACIÓN DEL PAR EN LOS ESLABONES CON RESPECTO A SUS VARIABLES DINÁMICAS')

%% Par 1
% Reemplazo de variables articulares
t1_n  = subs(t1,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

% Reemplazo de masas
t1_n  = subs(t1_n,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
t1_n  = subs(t1_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
t1_n  = subs(t1_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
t1_n  = subs(t1_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

fprintf("Pares hallados mediante Newton-Euler->\n")
fprintf("Par aplicado en el eslabón 1 (N.m):\n")
disp(double(t1_n)) % N.m
fprintf("Par aplicado en el eslabón 2 (kg.cm):\n")
disp(double(t2_n_valor)*10.2)
fprintf("Par aplicado en el eslabón 3 (kg.cm):\n")
disp(double(t3_n_valor)*10.2)
fprintf("Par aplicado en el eslabón 4 (kg.cm):\n")
disp(double(t4_n_valor)*10.2)
fprintf("Par aplicado en el eslabón 5 (kg.cm):\n")
disp(double(t5_n)*10.2)

%% ALGORITMO COMPUTACIONAL DE LAGRANGE-EULER PARA EL MODELADO DINÁMICO DEL ROBOT
% No considera la carga en el extremo del robot
% Matriz de fuerzas de inercias
Jv1=[diff(P_01,q1) diff(P_01,q2) diff(P_01,q3) diff(P_01,q4) diff(P_01,q5)];
Jv2=[diff(P_02,q1) diff(P_02,q2) diff(P_02,q3) diff(P_02,q4) diff(P_02,q5)];
Jv3=[diff(P_03,q1) diff(P_03,q2) diff(P_03,q3) diff(P_03,q4) diff(P_03,q5)];
Jv4=[diff(P_04,q1) diff(P_04,q2) diff(P_04,q3) diff(P_04,q4) diff(P_04,q5)];
Jv5=[diff(P_05,q1) diff(P_05,q2) diff(P_05,q3) diff(P_05,q4) diff(P_05,q5)];

Jv1_T=Jv1.';
Jv2_T=Jv2.';
Jv3_T=Jv3.';
Jv4_T=Jv4.';
Jv5_T=Jv5.';

Jw1=[e1*z0, zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1)];
Jw2=[e1*z0, e2*z1, zeros(3,1), zeros(3,1), zeros(3,1)];
Jw3=[e1*z0, e2*z1, e3*z2, zeros(3,1), zeros(3,1)];
Jw4=[e1*z0, e2*z1, e3*z2, e4*z3, zeros(3,1)];
Jw5=[e1*z0, e2*z1, e3*z2, e4*z3, e5*z4];

D = (m_base+m1_eje)*(Jv1.')*Jv1+(Jw1.')*R_01*I1_1*(R_01.')*Jw1+...
    (2*m_base+m2)*(Jv2.')*Jv2+(Jw2.')*R_02*I2_2*(R_02.')*Jw2+...
    (2*m_base+m3)*(Jv3.')*Jv3+(Jw3.')*R_03*I3_3*(R_03.')*Jw3+...
    (2*m_base+m4)*(Jv4.')*Jv4+(Jw4.')*R_04*I4_4*(R_04.')*Jw4+...
    (m_efector+m_servo_efector)*(Jv5.')*Jv5+(Jw5.')*R_05*I5_5*(R_05.')*Jw5;

% Energía cinética
q=[q1;q2;q3;q4;q5];
dq=[dq1;dq2;dq3;dq4;dq5];
ddq=[ddq1;ddq2;ddq3;ddq4;ddq5];

E_c = 1/2*dq.'*D*dq; 

% Matriz Coriolis
C  = matrizCoriolis(D, q, dq);

G1=-Jv1_T(1,1:3)*(m_base+m1_eje)*G - Jv2_T(1,1:3)*(2*m_base+m2)*G - Jv3_T(1,1:3)*(2*m_base+m3)*G...
    - Jv4_T(1,1:3)*(2*m_base+m4)*G - Jv5_T(1,1:3)*(m_efector+m_servo_efector)*G;
G2=-Jv1_T(2,1:3)*(m_base+m1_eje)*G - Jv2_T(2,1:3)*(2*m_base+m2)*G - Jv3_T(2,1:3)*(2*m_base+m3)*G...
    - Jv4_T(2,1:3)*(2*m_base+m4)*G - Jv5_T(2,1:3)*(m_efector+m_servo_efector)*G;
G3=-Jv1_T(3,1:3)*(m_base+m1_eje)*G - Jv2_T(3,1:3)*(2*m_base+m2)*G - Jv3_T(3,1:3)*(2*m_base+m3)*G...
    - Jv4_T(3,1:3)*(2*m_base+m4)*G - Jv5_T(3,1:3)*(m_efector+m_servo_efector)*G;
G4=-Jv1_T(4,1:3)*(m_base+m1_eje)*G - Jv2_T(4,1:3)*(2*m_base+m2)*G - Jv3_T(4,1:3)*(2*m_base+m3)*G...
    - Jv4_T(4,1:3)*(2*m_base+m4)*G - Jv5_T(4,1:3)*(m_efector+m_servo_efector)*G;
G5=-Jv1_T(5,1:3)*(m_base+m1_eje)*G - Jv2_T(5,1:3)*(2*m_base+m2)*G - Jv3_T(5,1:3)*(2*m_base+m3)*G...
    - Jv4_T(5,1:3)*(2*m_base+m4)*G - Jv5_T(5,1:3)*(m_efector+m_servo_efector)*G;

GG=[G1; G2; G3; G4; G5];

Tau_D=D*ddq+C*dq+GG;

%% Reemplazo numérico para el cálculo dinámico de Lagrange-Euler

% Reemplazo de variables articulares
Tau_D_n  = subs(Tau_D,[q1 q2 q3 q4 q5 dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5],...
    [q1_n q2_n q3_n q4_n q5_n dq1_n dq2_n dq3_n dq4_n dq5_n ddq1_n ddq2_n ddq3_n ddq4_n ddq5_n]);

% Reemplazo de masas
Tau_D_n  = subs(Tau_D_n,[m1_eje m2 m3 m4 m_efector m_servo_efector m_base m_servo],...
    [m1_eje_n m2_n m3_n m4_n m_efector_n m_servo_efector_n m_base_n m_servo_n]);

% Reemplazo de constantes
Tau_D_n  = subs(Tau_D_n,[pi g],...
    [pi_n g_n]);

% Reemplazo de longitudes
Tau_D_n  = subs(Tau_D_n,[L1_eje Lc2 Lc3 Lc4 L_servo_largo L_base L_servo_bracket L1 L2 L3 L4 L5 ...
   L_efector_base L_efector_final],...
    [L1_eje_n Lc2_n Lc3_n Lc4_n L_servo_largo_n L_base_n L_servo_bracket_n L1_n L2_n L3_n ...
    L4_n L5_n L_efector_base_n L_efector_final_n]);

% Reemplazo de momentos de inercia
Tau_D_n  = subs(Tau_D_n,[I1xx I1yy I1zz I2xx I2yy I2zz I3xx I3yy I3zz I4xx I4yy I4zz I5xx I5yy I5zz],...
    [I1xx_n I1yy_n I1zz_n I2xx_n I2yy_n I2zz_n I3xx_n I3yy_n I3zz_n I4xx_n I4yy_n I4zz_n I5xx_n I5yy_n I5zz_n]);

Tau_D_n_z = double(Tau_D_n); 
fprintf("Pares hallados mediante Lagrange-Euler (kg.cm)->\n")
disp(Tau_D_n_z*10.2) % kg.cm

% ################ PLANIFICACIÓN DE TRAYECTORIA ##########################














