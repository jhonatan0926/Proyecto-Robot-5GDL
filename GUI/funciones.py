import numpy as np
import math as mt
import sympy as sp
import time

#Funciones primordiales

def denavit(theta, d, a, alpha):

    # Matriz de rotación alrededor del eje z (theta)
    tz_theta = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,             0,              1, 0],
        [0,             0,              0, 1]
    ])
    
    # Matriz de traslación a lo largo del eje z (d)
    tz_d = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])
    
    # Matriz de traslación a lo largo del eje x (a)
    tx_a = np.array([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Matriz de rotación alrededor del eje x (alpha)
    tx_alpha = np.array([
        [1, 0,               0,                0],
        [0, np.cos(alpha), -np.sin(alpha), 0],
        [0, np.sin(alpha),  np.cos(alpha), 0],
        [0, 0,               0,                1]
    ])
    
    # Multiplicación de las matrices en el orden correcto
    T = tz_theta @ tz_d @ tx_a @ tx_alpha
    
    return T

def euler_angles_from_rotation_matrix(R):

    # Verificar que la matriz sea 3x3
    if R.shape != (3, 3):
        raise ValueError("La matriz de rotación debe ser de tamaño 3x3.")
    
    # Extraer los elementos relevantes de la matriz
    r11, r12, r13 = R[0, 0], R[0, 1], R[0, 2]
    r21, r22, r23 = R[1, 0], R[1, 1], R[1, 2]
    r31, r32, r33 = R[2, 0], R[2, 1], R[2, 2]
    
    # Calcular el ángulo de pitch (rotación alrededor de Y)
    pitch = np.arctan2(-r31, np.sqrt(r11**2 + r21**2))
    
    # Calcular los ángulos de yaw y roll
    if np.abs(np.cos(pitch)) > 1e-6:  # Evitar singularidad (gimbal lock)
        yaw = np.arctan2(r21 / np.cos(pitch), r11 / np.cos(pitch))
        roll = np.arctan2(r32 / np.cos(pitch), r33 / np.cos(pitch))
    else:
        # Gimbal lock: pitch = ±90°
        yaw = 0  # Arbitrario, ya que yaw y roll son indistinguibles
        roll = np.arctan2(r12, r22)
    
    return float(roll*180/mt.pi), float(pitch*180/mt.pi), float(yaw*180/mt.pi), 

#Funciones derivadas

def interpol_3(n_puntos, t0, tf, q0, qf, dq0, dqf):
    M = np.array([[t0**3, t0**2, t0, 1], 
                  [tf**3, tf**2, tf, 1], 
                  [3*(t0**2), 2*t0, 1, 0],
                  [3*(tf**2), 2*tf, 1, 0]])
    N = np.array([[q0], [qf], [dq0], [dqf]])
    coef = np.dot(np.linalg.inv(M), N)

    q_s = []
    dq_s = []
    ddq_s = []
    #Calulo de los puntos:
    for i in range(n_puntos+2):
        t_i = (tf - t0)/(n_puntos+2)
        t = t0 + i*t_i
        q = coef[0,0]*(t**3) + coef[1,0]*(t**2) + coef[2,0]*(t) +coef[3,0]
        dq = 3*coef[0,0]*(t**2) + 2*coef[1,0]*(t) + coef[2,0]
        ddq = 6*coef[0,0]*(t) + 2*coef[1,0]

        q_s.append(float(q))
        dq_s.append(float(dq))
        ddq_s.append(float(ddq))

    return [q_s, dq_s, ddq_s]

def interpol_5(n_puntos, t0, tf, q0, qf, dq0, dqf, ddq0, ddqf):
    M = np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1], 
                  [tf**5, tf**4, tf**3, tf**2, tf, 1], 
                  [5*(t0**4), 4*(t0**3), 3*(t0**2), 2*(t0), 1, 0],
                  [5*(tf**4), 4*(tf**3), 6*(tf**2), 2*(tf), 1, 0],
                  [20*(t0**3), 12*(t0**2), 6*t0, 2, 0, 0],
                  [20*(tf**3), 12*(tf**2), 6*tf, 2, 0, 0]])
    N = np.array([[q0], [qf], [dq0], [dqf], [ddq0], [ddqf]])
    coef = np.dot(np.linalg.inv(M), N)

    q_s = []
    dq_s = []
    ddq_s = []
    #Calulo de los puntos:
    for i in range(n_puntos+2):
        t_i = (tf - t0)/(n_puntos+2)
        t = t0 + i*t_i
        q = coef[0,0]*(t**5) + coef[1,0]*(t**4) + coef[2,0]*(t**3) + coef[3,0]*(t**2) + coef[4,0]*(t) + coef[5,0]
        dq = 5*coef[0,0]*(t**4) + 4*coef[1,0]*(t**3) + 3*coef[2,0]*(t**2) + 2*coef[3,0]*(t) + coef[4,0]
        ddq = 20*coef[0,0]*(t**3) + 12*coef[1,0]*(t**2) + 6*coef[2,0]*(t) + 2*coef[3,0]

        q_s.append(float(q))
        dq_s.append(float(dq))
        ddq_s.append(float(ddq))

    return [q_s, dq_s, ddq_s]

def cin_inversa(L1,L2,L3,L4,p_x,p_y,p_z,phi,theta,psi):
    estado = 0
    #Calculo de las angulos
    phi = phi*mt.pi/180
    theta = theta*mt.pi/180
    psi = psi*mt.pi/180
    #Calculo de R n_z o_z a_z 
    Rx = [[1, 0, 0], [0, mt.cos(phi), -mt.sin(phi)], [0, mt.sin(phi), mt.cos(phi)]]
    Ry = [[mt.cos(theta), 0, mt.sin(theta)], [0, 1, 0], [-mt.sin(theta), 0, mt.cos(theta)]]
    Rz = [[mt.cos(psi), -mt.sin(psi), 0], [mt.sin(psi), mt.cos(psi), 0], [0, 0, 1]]
    R = np.dot(Rz,np.dot(Ry,Rx))

    n_z = R[2,0]
    o_z = R[2,1]
    a_z = R[2,2]

    q_1 = mt.atan(p_y/p_x)
    q_5 = mt.atan(-o_z/n_z)
    q_234 = mt.asin(a_z)

    #Correccion de los valores de q_1 q_5 q_234 
    if q_1<0:
        q_1 = q_1 + mt.pi
    
    if q_5<0:
        q_5 = q_5 + mt.pi

    if q_234<0:
        #q_234 < 45*mt.pi/180
        q_234 = mt.pi - q_234

    # Definir la variable simbólica
    x = sp.symbols('x')
    q_2 = 0
    q_3 = 0
    q_4 = 0

    # Iterar sobre el rango de ángulos
    for ang in np.arange(0, 180.5, 0.5):
        q_2 = ang * np.pi/180  #Convertir a radianes
        #Definir la ecuación simbólica
        eq = L1 + L3*sp.sin(q_2 + x) + L2*sp.sin(q_2) + L4*sp.sin(q_234) - p_z
        
        # Resolver la ecuación
        solu = sp.solve(eq, x)
        
        if solu:  # Verificar si hay soluciones
            for n in range(len(solu)):
                # Verificar si la solución es real
                if solu[n].is_real:  # Filtra solo soluciones reales
                    q_3 = solu[n]  # Tomar la solución
                    q_4 = q_234 - q_2 - q_3
                    
                    # Verificar si q_3 y q_4 son positivos
                    if float(q_4) > 0 and float(q_3) > 0:
                        estado = 1
                        break
            if estado == 1:
                #Conversion a grados sexagesimales
                q_1 = float(q_1*180/mt.pi)
                q_2 = float(q_2*180/mt.pi)
                q_3 = float(q_3*180/mt.pi)
                q_4 = float(q_4*180/mt.pi)
                q_5 = float(q_5*180/mt.pi)

                print([q_1, q_2, q_3, q_4, q_5])
                return [q_1, q_2, q_3, q_4, q_5]
                #return [float(q_1*180/mt.pi), float(q_2*180/mt.pi), 
                        #float(q_3*180/mt.pi), float(q_4*180/mt.pi), float(q_5*180/mt.pi)]

def enviar_angulos(ang1, ang2, ang3, ang4, ang5, arduino):  # Cambia 'COM3' por el puerto correcto
    time.sleep(2)  # Esperar a que se establezca la conexión
    # Crear una cadena con los ángulos separados por comas
    datos = f"{ang1},{ang2},{ang3},{ang4},{ang5}\n"
    
    # Enviar los datos al Arduino
    arduino.write(datos.encode())
    #print(f"Ángulos enviados: {datos.strip()}")