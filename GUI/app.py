import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import math as mt
import serial
from PIL import Image, ImageTk
import webbrowser

#Intento de conectarse a arduino
#------------------------------------------------------------------------------
try:
    arduino = serial.Serial('COM10', 9600)
except:
    arduino = None
#------------------------------------------------------------------------------

from funciones import interpol_3, interpol_5, cin_inversa, enviar_angulos
###########################################################################

#Configuraciones del robot
config = "config.txt"
# Lista para almacenar los puntos
puntos = []
# Lista para almacenar los angulos interpolados
angulos_interpol = [[],[],[],[],[]]
angulos_interpol_ang = [[],[],[],[],[]]
# Lista para almacenar los velocidades interpolados
velocidades_interpol = [[],[],[],[],[]]
velocidades_interpol_ang = [[],[],[],[],[]]
# Lista para almacenar los velocidades interpolados
aceleraciones_interpol = [[],[],[],[],[]]
aceleraciones_interpol_ang = [[],[],[],[],[]]
#Lista de los sliders
sliders_ant = [0,90,90,90,90]
sliders = []
#Valores de los eslabones
L1 = 235/1000
L2 = 182/1000
L3 = 202/1000
L4 = 218.8/1000

# Función para leer los valores del archivo
def leer_valores():
    try:
        with open(config, "r") as archivo:
            lineas = archivo.readlines()

        valores = {}
        for linea in lineas:
            if ":" in linea:
                clave, valor = linea.split(":")
                valores[clave.strip()] = valor.strip().split(" ")[0]
        return valores
    except FileNotFoundError:
        messagebox.showerror("Error", "El archivo de configuración no existe.")
        return None

# Función para guardar los valores en el archivo
def guardar_valores():
    try:
        with open(config, "w") as archivo:
            archivo.write("Configuración de motores\n")
            archivo.write("========================\n\n")
            #archivo.write(f"Velocidad máxima q1: {entry_vel1.get()} deg/s\n")
            #archivo.write(f"Velocidad máxima q2: {entry_vel2.get()} deg/s\n")
            #archivo.write(f"Aceleración máxima q1: {entry_ac1.get()} deg/s²\n")
            #archivo.write(f"Aceleración máxima q2: {entry_ac2.get()} deg/s²\n")
        messagebox.showinfo("Guardado", "Los valores se han guardado correctamente.")
    except Exception as e:
        messagebox.showerror("Error", f"No se pudo guardar el archivo: {e}")

# Función para agregar puntos
def agregar_punto():
    try:
        x = float(entry_x.get())
        y = float(entry_y.get())
        z = float(entry_z.get())
        puntos.append((x, y, z))
        entry_x.delete(0, tk.END)
        entry_y.delete(0, tk.END)
        entry_z.delete(0, tk.END)
        tabla.insert("", "end", values=(x, y, z))
        messagebox.showinfo("Éxito", f"Punto ({x}, {y}, {z}) agregado.")
    except ValueError:
        messagebox.showerror("Error", "Ingrese valores numéricos válidos.")

# Función para cargar puntos desde un archivo Excel
def usar_xlsx():
    file_path = filedialog.askopenfilename(filetypes=[("Excel files", "*.xlsx")])
    if file_path:
        try:
            df = pd.read_excel(file_path)
            puntos.extend(df.values.tolist())
            # Añadir los puntos a la tabla
            for punto in df.values:
                tabla.insert("", "end", values=tuple(punto))
            messagebox.showinfo("Éxito", "Puntos cargados desde el archivo Excel.")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo cargar el archivo: {e}")

# Función para calcular puntos intermedios
def calcular_puntos():
    try:
        num_puntos = int(entry_num_puntos.get())
        if len(puntos) < 2:
            messagebox.showwarning("Advertencia", "Debe haber al menos 2 puntos para calcular intermedios.")
            return
        
        # Calculo de la cinemática inversa
        mtr_qs = []

        for i in range(len(puntos)):
            print(f"intento{i}")
            qs = cin_inversa(L1, L2, L3, L4, puntos[i][0],puntos[i][1],puntos[i][2],0,-180,0)
            mtr_qs.append(qs)

        # Interpolación lineal simple entre los puntos
        print(mtr_qs)
        for i in range(len(puntos)-1):
            entry_num_puntos.get()
            tipo=combo_tipo.get()
            t_i = t_pp*i
            t_f = t_pp*(i+1)
            if tipo == "Splin cúbico":
                q1_vals = interpol_3(num_puntos, t_i, t_f, mtr_qs[i][0], mtr_qs[i+1][0], 10, 5)
                q2_vals = interpol_3(num_puntos, t_i, t_f, mtr_qs[i][1], mtr_qs[i+1][1], 20, 6)
                q3_vals = interpol_3(num_puntos, t_i, t_f, mtr_qs[i][2], mtr_qs[i+1][2], 20, 4)
                q4_vals = interpol_3(num_puntos, t_i, t_f, mtr_qs[i][3], mtr_qs[i+1][3], 20, 6)
                q5_vals = interpol_3(num_puntos, t_i, t_f, mtr_qs[i][4], mtr_qs[i+1][4], 20, 4)
            elif tipo == "Splin quíntico":
                q1_vals = interpol_5(num_puntos, t_i, t_f, mtr_qs[i][0], mtr_qs[i+1][0], 10, 5, 1, 2)
                q2_vals = interpol_5(num_puntos, t_i, t_f, mtr_qs[i][1], mtr_qs[i+1][1], 20, 6, 1, 2)
                q3_vals = interpol_5(num_puntos, t_i, t_f, mtr_qs[i][2], mtr_qs[i+1][2], 20, 4, 1, 2)
                q4_vals = interpol_5(num_puntos, t_i, t_f, mtr_qs[i][3], mtr_qs[i+1][3], 20, 6, 1, 2)
                q5_vals = interpol_5(num_puntos, t_i, t_f, mtr_qs[i][4], mtr_qs[i+1][4], 20, 4, 1, 2)

            angulos_interpol[0] += q1_vals[0]
            angulos_interpol[1] += q2_vals[0]
            angulos_interpol[2] += q3_vals[0]
            angulos_interpol[3] += q4_vals[0]
            angulos_interpol[4] += q5_vals[0]

            velocidades_interpol[0] += q1_vals[1]
            velocidades_interpol[1] += q2_vals[1]
            velocidades_interpol[2] += q3_vals[1]
            velocidades_interpol[3] += q4_vals[1]
            velocidades_interpol[4] += q5_vals[1]

            aceleraciones_interpol[0] += q1_vals[2]
            aceleraciones_interpol[1] += q2_vals[2]
            aceleraciones_interpol[2] += q3_vals[2]
            aceleraciones_interpol[3] += q4_vals[2]
            aceleraciones_interpol[4] += q5_vals[2]

        messagebox.showinfo("Éxito", f"{num_puntos} puntos intermedios calculados.")
    except ValueError:
        messagebox.showerror("Error", "Ingrese un número válido de puntos.")

# Función para mover el robot usando un archivo de texto
def mover_con_txt():
    file_path = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")])
    if file_path:
        try:
            with open(file_path, "r") as file:
                lines = file.readlines()
                for line in lines:
                    coords = line.strip().split(",")
                    punto = (float(coords[0]), float(coords[1]), float(coords[2]))
                    puntos.append(punto)
                    tabla.insert("", "end", values=punto)
            messagebox.showinfo("Éxito", "Puntos cargados desde el archivo de texto.")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo cargar el archivo: {e}")

# Función para mover el robot
def mover_robot():
    if not puntos:
        messagebox.showwarning("Advertencia", "No hay puntos para mover el robot.")
        return
    messagebox.showinfo("Mover Robot", "El robot se está moviendo según la trayectoria planificada.")

# Función para realizar gráficas
def graficar_trayectoria():
    if not puntos:
        messagebox.showwarning("Advertencia", "No hay puntos para graficar.")
        return

    # Limpiar la figura anterior
    ax.clear()
    print(puntos)
    # Obtener las coordenadas de los puntos
    x_vals = [p[0] for p in puntos]
    y_vals = [p[1] for p in puntos]
    z_vals = [p[2] for p in puntos]

    # Graficar la trayectoria
    ax.plot(x_vals, y_vals, z_vals, marker='x', color='blue')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Trayectoria del Robot en 3D")

    # Cambiar el color de fondo de los ejes y la figura
    ax.set_facecolor('#f0f0f0')  # Color de fondo de los ejes (gris claro)
    fig.patch.set_facecolor('#f0f0f0')  # Color de fondo de la figura (blanco)

    # Redibujar el canvas
    canvas.draw()

def graficar_angulos():
    if not puntos:
        messagebox.showwarning("Advertencia", "No hay puntos para graficar ángulos.")
        return

    # Limpiar las figuras anteriores
    for ax in axs:
        ax.clear()

    # calcular  matriz de tiempos
    tiempo = np.linspace(0, (len(puntos)-1)*t_pp, len(angulos_interpol[0]))
    print(tiempo)
    print(angulos_interpol[0])

    # Graficar cada ángulo
    for i, ax in enumerate(axs):
        ax.plot(tiempo, angulos_interpol[i], label=f"q {i + 1}")
        ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel(f'Ángulo {i + 1} (deg)')
        ax.legend()
        ax.grid(True)

    # Redibujar el canvas de ángulos
    canvas_angulos.draw()

def graficar_velocidades():
    if not puntos:
        messagebox.showwarning("Advertencia", "No hay puntos para graficar velocidades.")
        return

    # Limpiar las figuras anteriores
    for ax in axs_v:
        ax.clear()

    # Simular velocidades para cada grado de libertad (5GDL)
    tiempo = np.linspace(0, (len(puntos)-1)*t_pp, len(angulos_interpol[0]))
    print(tiempo)
    print(angulos_interpol[0])

    # Graficar cada velocidad
    for i, ax in enumerate(axs_v):
        ax.plot(tiempo, velocidades_interpol[i], label=f"dq {i + 1}")
        ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel(f'Velocidad ang {i + 1} (deg/s)')
        ax.legend()
        ax.grid(True)

    # Redibujar el canvas de velocidades
    canvas_velocidades.draw()

def graficar_aceleraciones():
    if not puntos:
        messagebox.showwarning("Advertencia", "No hay puntos para graficar aceleraciones.")
        return

    # Limpiar las figuras anteriores
    for ax in axs_a:
        ax.clear()

    # Simular aceleraciones para cada grado de libertad (5GDL)
    tiempo = np.linspace(0, (len(puntos)-1)*t_pp, len(angulos_interpol[0]))
    print(tiempo)
    print(angulos_interpol[0])

    # Graficar cada aceleracion
    for i, ax in enumerate(axs_a):
        ax.plot(tiempo, aceleraciones_interpol[i], label=f"ddq {i + 1}")
        ax.set_xlabel('Tiempo (s)')
        ax.set_ylabel(f'Aceleración ang {i + 1} (deg/s2)')
        ax.legend()
        ax.grid(True)

    # Redibujar el canvas de aceleraciones
    canvas_aceleraciones.draw()

def graficar():
    graficar_angulos()
    graficar_velocidades()
    graficar_aceleraciones()
    print(f"angulo q1: {len(angulos_interpol[0])} elementos")
    print(list(angulos_interpol[0]))
    print(f"angulo q2: {len(angulos_interpol[1])} elementos")
    print(list(angulos_interpol[1]))
    print(f"angulo q3: {len(angulos_interpol[2])} elementos")
    print(list(angulos_interpol[2]))
    print(f"angulo q4: {len(angulos_interpol[3])} elementos")
    print(list(angulos_interpol[3]))
    print(f"angulo q5: {len(angulos_interpol[4])} elementos")
    print(list(angulos_interpol[4]))

# Función para cerrar la aplicación correctamente
def cerrar_aplicacion():
    plt.close('all')  # Cierra todas las figuras de matplotlib
    root.quit()       # Cierra la ventana de tkinter

#Resetear los valores
def borrar_calculos():
    puntos.clear()
    angulos_interpol.clear()
    velocidades_interpol.clear()
    aceleraciones_interpol.clear()

def exportar_txt():
    # Nombre del archivo de salida
    nombre_archivo = "angulos.txt"

    # Exportar la lista a un archivo txt
    with open(nombre_archivo, "w") as archivo:
        for sublista in angulos_interpol:
            # Convertir cada sublista a una cadena separada por comas
            linea = ",".join(sublista)
            # Escribir la línea en el archivo
            archivo.write(linea + "\n")

    messagebox.showinfo("Exportado", "Los valores se han exportado correctamente.")

def interpol_ang(tipo_interpol, num_puntos):

    global sliders, sliders_ant

    sliders += [float(slider1.get()), float(slider2.get()), float(slider3.get()), float(slider4.get()), float(slider5.get())]
    
    if tipo_interpol == "Splin cúbico":
        q1_vals = interpol_3(num_puntos, 0, 1, sliders_ant[0], sliders[0], 10, 5)
        q2_vals = interpol_3(num_puntos, 0, 1, sliders_ant[1], sliders[1], 20, 6)
        q3_vals = interpol_3(num_puntos, 0, 1, sliders_ant[2], sliders[2], 5, 4)
        q4_vals = interpol_3(num_puntos, 0, 1, sliders_ant[3], sliders[3], 20, 6)
        q5_vals = interpol_3(num_puntos, 0, 1, sliders_ant[4], sliders[4], 5, 4)
    elif tipo_interpol == "Splin quíntico":
        q1_vals = interpol_5(num_puntos, 0, 1, sliders_ant[0], sliders[0], 10, 5, 1, 2)
        q2_vals = interpol_5(num_puntos, 0, 1, sliders_ant[1], sliders[1], 20, 6, 1, 2)
        q3_vals = interpol_5(num_puntos, 0, 1, sliders_ant[2], sliders[2], 5, 4, 1, 2)
        q4_vals = interpol_5(num_puntos, 0, 1, sliders_ant[3], sliders[3], 20, 6, 1, 2)
        q5_vals = interpol_5(num_puntos, 0, 1, sliders_ant[4], sliders[4], 5, 4, 1, 2)

    angulos_interpol_ang[0].clear()
    angulos_interpol_ang[0] += q1_vals[0]
    angulos_interpol_ang[1].clear()
    angulos_interpol_ang[1] += q2_vals[0]
    angulos_interpol_ang[2].clear()
    angulos_interpol_ang[2] += q3_vals[0]
    angulos_interpol_ang[3].clear()
    angulos_interpol_ang[3] += q4_vals[0]
    angulos_interpol_ang[4].clear()
    angulos_interpol_ang[4] += q5_vals[0]


    sliders_ant.clear()
    print(len(sliders_ant))
    sliders_ant += sliders
    sliders.clear()
    print(len(sliders))

def mover_2pts(list_ang):
    print(list_ang)
    k = len(list_ang[0])
    for i in range(k):
        print(i)
        enviar_angulos(list_ang[0][i], list_ang[1][i], list_ang[2][i], list_ang[3][i], list_ang[4][i], arduino)
###############################################################
###############################################################
###############################################################
###############################################################

config = leer_valores()
vel_max = [float(config["vel_1_max"]), float(config["vel_2_max"]), float(config["vel_3_max"]), float(config["vel_4_max"]), float(config["vel_5_max"])]
acel_max = [float(config["acel_1_max"]), float(config["acel_2_max"]), float(config["acel_3_max"]), float(config["acel_4_max"]), float(config["acel_5_max"])]
t_pp = float(config["tiempo"])

###############################################################
###############################################################
###############################################################
###############################################################

# Crear la ventana principal
root = tk.Tk()
root.title("Planificación de trayectoria de robot de 5GDL")
icono = tk.PhotoImage(file="logo_robot.png")
root.iconphoto(True, icono)
#root.geometry("800x600") #Dimensiones ventana
root.state('zoomed') #Ventana maximizada
root.attributes('-fullscreen', True) #Pantalla completa
root.bind('<Escape>', lambda e: root.attributes('-fullscreen', False))

# Manejar el cierre de la ventana
root.protocol("WM_DELETE_WINDOW", cerrar_aplicacion)

# Crear un Notebook (pestañas)
notebook = ttk.Notebook(root)
notebook.pack(fill=tk.BOTH, expand=True)

# Frame principal
frame = ttk.Frame(notebook)
notebook.add(frame, text="Planificación de Trayectoria")

# Campos para ingresar coordenadas
ttk.Label(frame, text="Coordenadas:").place(x=10, y=30)
ttk.Label(frame, text="x:").place(x=100, y=30)
entry_x = ttk.Entry(frame, width=10)
entry_x.place(x=120, y=30)
ttk.Label(frame, text="y:").place(x=200, y=30)
entry_y = ttk.Entry(frame, width=10)
entry_y.place(x=220, y=30)
ttk.Label(frame, text="z:").place(x=300, y=30)
entry_z = ttk.Entry(frame, width=10)
entry_z.place(x=320, y=30)

# Botón para agregar puntos
ttk.Button(frame, text="Agregar", command=agregar_punto).place(x=420, y=30)
# Botón para usar archivo Excel
ttk.Button(frame, text="Usar xlsx", command=usar_xlsx).place(x=510, y=30)

# Campo para ingresar el número de puntos
ttk.Label(frame, text="Número de puntos a generar:").place(x=10, y=80)
entry_num_puntos = ttk.Entry(frame, width=10)
entry_num_puntos.place(x=180, y=80)

# Campo para ingresar el grado de interpolación
ttk.Label(frame, text="Tipo de interpolación:").place(x=10, y=130)
tipos_interpolacion = ["Splin cúbico", "Splin quíntico"]
# Crear la caja de selección (Combobox)
combo_tipo = ttk.Combobox(frame, values=tipos_interpolacion, width=13)
combo_tipo.place(x=150, y=130)
combo_tipo.current(0)  # Establecer el valor predeterminado (primer elemento de la lista)

# Botón para calcular puntos intermedios
ttk.Button(frame, text="Calcular puntos", command=calcular_puntos).place(x=270, y=130)

# Botón para graficar la trayectoria
ttk.Button(frame, text="Graficar Trayectoria", command=graficar_trayectoria).place(x=10, y=180)

# Botón para mover con archivo de texto
ttk.Button(frame, text="Mover con txt", command=mover_con_txt).place(x=150, y=180)

# Botón para mover el robot
ttk.Button(frame, text="Mover robot", command=mover_robot).place(x=270, y=180)

# Mostrar puntos ingresados
ttk.Label(frame, text="Puntos ingresados:").place(x=10, y=230)

# Crear una tabla para mostrar los puntos
columnas = ("x", "y", "z")
tabla = ttk.Treeview(frame, columns=columnas, show="headings")
tabla.heading("x", text="X")
tabla.heading("y", text="Y")
tabla.heading("z", text="Z")

tabla.place(x=30, y=280, width=535, height=380)

# Crear una figura de matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Trayectoria del Robot en 3D")

# Configurar colores de fondo iniciales
ax.set_facecolor('#f0f0f0')  # Color de fondo de los ejes
fig.patch.set_facecolor('#f0f0f0')  # Color de fondo de la figura

# Crear un canvas de matplotlib para integrarlo en tkinter
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.get_tk_widget().place(x=610, y=20, width=770, height=680)

###############################################################
###############################################################
###############################################################
###############################################################

# Crear una nueva pestaña para las gráficas de ángulos
tab_angulos = ttk.Frame(notebook)
notebook.add(tab_angulos, text="Ángulos")

# Crear una figura de matplotlib para los ángulos
fig_angulos, axs = plt.subplots(5, 1, figsize=(15, 10))
fig_angulos.patch.set_facecolor('#f0f0f0')  # Color de fondo de la figura

# Crear un canvas de matplotlib para integrarlo en la pestaña de ángulos
canvas_angulos = FigureCanvasTkAgg(fig_angulos, master=tab_angulos)
canvas_angulos.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Botón para graficar los ángulos
ttk.Button(frame, text="Graficar Ang, Vel y Ac", command=graficar).place(x=390, y=180)

###############################################################
###############################################################
###############################################################
###############################################################

# Crear una nueva pestaña para las gráficas de las velocidades
tab_velocidades = ttk.Frame(notebook)
notebook.add(tab_velocidades, text="Velocidades")

# Crear una figura de matplotlib para los velocidades
fig_velocidades, axs_v = plt.subplots(5, 1, figsize=(15, 10))
fig_velocidades.patch.set_facecolor('#f0f0f0')  # Color de fondo de la figura

# Crear un canvas de matplotlib para integrarlo en la pestaña de velocidades
canvas_velocidades = FigureCanvasTkAgg(fig_velocidades, master=tab_velocidades)
canvas_velocidades.get_tk_widget().pack(fill=tk.BOTH, expand=True)

###############################################################
###############################################################
###############################################################
###############################################################

# Crear una nueva pestaña para las gráficas de las aceleraciones
tab_aceleraciones = ttk.Frame(notebook)
notebook.add(tab_aceleraciones, text="Aceleraciones")

# Crear una figura de matplotlib para los aceleraciones
fig_aceleraciones, axs_a = plt.subplots(5, 1, figsize=(15, 10))
fig_aceleraciones.patch.set_facecolor('#f0f0f0')  # Color de fondo de la figura

# Crear un canvas de matplotlib para integrarlo en la pestaña de aceleraciones
canvas_aceleraciones = FigureCanvasTkAgg(fig_aceleraciones, master=tab_aceleraciones)
canvas_aceleraciones.get_tk_widget().pack(fill=tk.BOTH, expand=True)

###############################################################
###############################################################
###############################################################
###############################################################

# Crear una nueva pestaña para control manual
tab_manual = ttk.Frame(notebook)
notebook.add(tab_manual, text="Control manual")

# Función para actualizar el slider cuando se modifica el valor en el Entry
def update_slider_from_entry(slider, entry):
    try:
        value = int(entry.get())  # Obtener el valor del Entry
        if slider.cget("from") <= value <= slider.cget("to"):  # Verificar que esté dentro del rango
            slider.set(value)  # Actualizar el slider
    except ValueError:
        pass  # Ignorar si el valor no es un número válido

# Función para actualizar el Entry cuando se mueve el slider
def update_entry_from_slider(slider, entry):
    entry.delete(0, tk.END)  # Borrar el contenido actual del Entry
    entry.insert(0, str(slider.get()))  # Insertar el valor actual del slider

# Crear sliders (deslizadores) para controlar los ángulos

slider_length = 400  # Longitud de los sliders
entry_width = 5      # Ancho de los cuadros de texto

# Slider 1
slider1 = tk.Scale(tab_manual, from_=-90, to=90, orient=tk.HORIZONTAL, label="Ángulo q1", length=slider_length)
slider1.set(0)  # Valor inicial
slider1.place(x=30, y=30)

entry1 = tk.Entry(tab_manual, width=entry_width)  # Cuadro de texto para el valor
entry1.insert(0, str(slider1.get()))  # Mostrar el valor inicial
entry1.place(x=450, y=65)

# Vincular el Entry al slider
slider1.configure(command=lambda value: update_entry_from_slider(slider1, entry1))
entry1.bind("<Return>", lambda event: update_slider_from_entry(slider1, entry1))

# Slider 2
slider2 = tk.Scale(tab_manual, from_=0, to=180, orient=tk.HORIZONTAL, label="Ángulo q2", length=slider_length)
slider2.set(90)  # Valor inicial
slider2.place(x=30, y=90)

entry2 = tk.Entry(tab_manual, width=entry_width)  # Cuadro de texto para el valor
entry2.insert(0, str(slider2.get()))  # Mostrar el valor inicial
entry2.place(x=450, y=125)

# Vincular el Entry al slider
slider2.configure(command=lambda value: update_entry_from_slider(slider2, entry2))
entry2.bind("<Return>", lambda event: update_slider_from_entry(slider2, entry2))

# Slider 3
slider3 = tk.Scale(tab_manual, from_=0, to=180, orient=tk.HORIZONTAL, label="Ángulo q3", length=slider_length)
slider3.set(90)  # Valor inicial
slider3.place(x=30, y=150)

entry3 = tk.Entry(tab_manual, width=entry_width)  # Cuadro de texto para el valor
entry3.insert(0, str(slider3.get()))  # Mostrar el valor inicial
entry3.place(x=450, y=185)

# Vincular el Entry al slider
slider3.configure(command=lambda value: update_entry_from_slider(slider3, entry3))
entry3.bind("<Return>", lambda event: update_slider_from_entry(slider3, entry3))

# Slider 4
slider4 = tk.Scale(tab_manual, from_=0, to=180, orient=tk.HORIZONTAL, label="Ángulo q4", length=slider_length)
slider4.set(90)  # Valor inicial
slider4.place(x=30, y=210)

entry4 = tk.Entry(tab_manual, width=entry_width)  # Cuadro de texto para el valor
entry4.insert(0, str(slider4.get()))  # Mostrar el valor inicial
entry4.place(x=450, y=245)

# Vincular el Entry al slider
slider4.configure(command=lambda value: update_entry_from_slider(slider4, entry4))
entry4.bind("<Return>", lambda event: update_slider_from_entry(slider4, entry4))

# Slider 5
slider5 = tk.Scale(tab_manual, from_=0, to=180, orient=tk.HORIZONTAL, label="Ángulo q5", length=slider_length)
slider5.set(90)  # Valor inicial
slider5.place(x=30, y=270)

entry5 = tk.Entry(tab_manual, width=entry_width)  # Cuadro de texto para el valor
entry5.insert(0, str(slider5.get()))  # Mostrar el valor inicial
entry5.place(x=450, y=305)

# Vincular el Entry al slider
slider5.configure(command=lambda value: update_entry_from_slider(slider5, entry5))
entry5.bind("<Return>", lambda event: update_slider_from_entry(slider5, entry5))

#Tipo de interpolacion
ttk.Label(tab_manual, text="Tipo de interpolación:").place(x=30, y=355)
# Crear la caja de selección (Combobox)
combo_tipo2 = ttk.Combobox(tab_manual, values=tipos_interpolacion, width=13)
combo_tipo2.place(x=200, y=355)
combo_tipo2.current(0)  # Establecer el valor predeterminado (primer elemento de la lista)

#Calculo de los puntos a generar
ttk.Label(tab_manual, text="Número de puntos a generar:").place(x=30, y=405)
entry_num_puntos2 = ttk.Entry(tab_manual, width=16)
entry_num_puntos2.place(x=200, y=405)

ttk.Button(tab_manual, text="Calcular", command=lambda : interpol_ang(combo_tipo2.get(), int(entry_num_puntos2.get()))).place(x=320, y=405)

#Puerto del arduino
ttk.Label(tab_manual, text="Puerto arduino:").place(x=30, y=455)
entry_x = ttk.Entry(tab_manual, width=16)
entry_x.place(x=200, y=455)

ttk.Button(tab_manual, text="Conectar").place(x=320, y=455)
# Botón para enviar los ángulos al Arduino
ttk.Button(tab_manual, text="Enviar Ángulos", width=70 , command= lambda: mover_2pts(angulos_interpol_ang)).place(x=40, y=505)

# Cargar la imagen
image = Image.open("Robot5GDL.png")  # Cambia la ruta a tu imagen
image = image.resize((600, 500))  # Redimensionar la imagen si es necesario
photo = ImageTk.PhotoImage(image)

# Crear un Label para mostrar la imagen
label = tk.Label(tab_manual, image=photo)
label.image = photo  # Mantener una referencia para evitar que la imagen sea eliminada por el recolector de basura
label.place(x=600, y=40)

###############################################################
###############################################################
###############################################################
###############################################################

# Crear una nueva pestaña para las gráficas de las aceleraciones
tab_acercade = ttk.Frame(notebook)
notebook.add(tab_acercade, text="Acerca de")

# Cargar la imagen
image = Image.open("logo_mecatronica.png")  # Cambia la ruta a tu imagen
image = image.resize((250, 250))  # Redimensionar la imagen si es necesario
photo = ImageTk.PhotoImage(image)

# Crear un Label para mostrar la imagen
label = tk.Label(tab_acercade, image=photo)
label.image = photo  # Mantener una referencia para evitar que la imagen sea eliminada por el recolector de basura
label.pack()

# Función para abrir el enlace en el navegador
def abrir_enlace():
    webbrowser.open("https://github.com/jhonatan0926/Proyecto-Robot-5GDL")

# Campos para ingresar coordenadas
label = tk.Label(tab_acercade, text="Aplicación utilizada para controlar el movimiento de un robot de 5GDL.").pack()

label = tk.Label(tab_acercade, text="Desarollado como parte del proyecto de la unidad 3 del curso de robótica").pack()
ttk.Button(tab_acercade, text="Visitar repositorio", command=abrir_enlace).pack()

###############################################################
###############################################################
###############################################################
###############################################################

# Iniciar el bucle principal de la ventana
root.mainloop()
