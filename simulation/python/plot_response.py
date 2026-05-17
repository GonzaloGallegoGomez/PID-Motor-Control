# ══════════════════════════════════════════════════════
#  PID Motor Control — Real-time data plotter
#  Gonzalo Gallego Gomez — EEBE UPC
#  Lee datos del Arduino por puerto serie y los grafica
#  en tiempo real usando matplotlib
# ══════════════════════════════════════════════════════

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

# ── CONFIGURACIÓN DEL PUERTO SERIE ─────────────────────
# Cambia 'COM3' por el puerto donde tengas el Arduino
# En Windows: COM3, COM4, COM5... (míralo en el Arduino IDE)
# En Mac/Linux: '/dev/ttyUSB0' o '/dev/ttyACM0'
PUERTO    = 'COM3'
BAUDIOS   = 9600   # debe coincidir con Serial.begin(9600) del Arduino
MAX_DATOS = 100    # cuántos puntos mostramos en la gráfica a la vez

# ── BUFFERS DE DATOS ────────────────────────────────────
# deque es como una lista pero con tamaño máximo fijo
# cuando se llena, elimina el dato más antiguo automáticamente
# es perfecta para gráficas de ventana deslizante
setpoint_data = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
rpm_data      = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
pid_data      = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
error_data    = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
tiempo_data   = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)

tiempo_inicio = time.time()  # guardamos cuando arrancó el script

# ── CONEXIÓN CON EL ARDUINO ─────────────────────────────
try:
    arduino = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    time.sleep(2)  # esperamos 2s a que el Arduino arranque
    print(f"Conectado al Arduino en {PUERTO}")
except Exception as e:
    print(f"Error al conectar: {e}")
    print("Comprueba que el Arduino está conectado y el puerto es correcto")
    exit()

# ── CONFIGURACIÓN DE LA GRÁFICA ─────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle('PID Motor Control — Real-time Response',
             fontsize=14, fontweight='bold')

# Gráfica superior: RPM actual vs Setpoint
ax1.set_ylabel('Speed (RPM)')
ax1.set_xlabel('Time (s)')
ax1.set_title('Motor Speed vs Setpoint')
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 200)  # ajusta según tus RPM objetivo

line_setpoint, = ax1.plot([], [], 'r--', linewidth=2,
                           label='Setpoint')
line_rpm,      = ax1.plot([], [], 'b-',  linewidth=2,
                           label='Actual RPM')
ax1.legend(loc='upper right')

# Gráfica inferior: error en tiempo real
ax2.set_ylabel('Error (RPM)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Control Error')
ax2.grid(True, alpha=0.3)
ax2.axhline(y=0, color='k', linewidth=0.5)  # línea de error cero

line_error, = ax2.plot([], [], 'g-', linewidth=2, label='Error')
ax2.legend(loc='upper right')

# ── FUNCIÓN DE LECTURA Y ACTUALIZACIÓN ──────────────────
def actualizar(frame):
    """
    Esta función se llama automáticamente cada 50ms
    Lee una línea del Arduino, parsea el CSV y actualiza
    las gráficas. Es equivalente a un callback en Python.
    """
    try:
        # Leemos una línea del puerto serie
        linea = arduino.readline().decode('utf-8').strip()

        # Ignoramos la cabecera CSV y líneas vacías
        if ',' not in linea or 'setpoint' in linea:
            return line_setpoint, line_rpm, line_error

        # Parseamos el CSV: setpoint, RPM, salidaPID, error
        valores = linea.split(',')
        if len(valores) != 4:
            return line_setpoint, line_rpm, line_error

        sp    = float(valores[0])  # setpoint
        rpm   = float(valores[1])  # RPM actuales
        pid   = float(valores[2])  # salida PID
        error = float(valores[3])  # error actual

        # Calculamos el tiempo transcurrido desde el inicio
        t = time.time() - tiempo_inicio

        # Añadimos los nuevos datos a los buffers
        setpoint_data.append(sp)
        rpm_data.append(rpm)
        pid_data.append(pid)
        error_data.append(error)
        tiempo_data.append(t)

        # Convertimos deque a lista para matplotlib
        t_list = list(tiempo_data)

        # Actualizamos las líneas de la gráfica
        line_setpoint.set_data(t_list, list(setpoint_data))
        line_rpm.set_data(t_list, list(rpm_data))
        line_error.set_data(t_list, list(error_data))

        # Ajustamos el eje X para que se desplace con el tiempo
        ax1.set_xlim(max(0, t - 10), t + 1)
        ax2.set_xlim(max(0, t - 10), t + 1)

        # Ajustamos el eje Y del error dinámicamente
        if len(error_data) > 1:
            margen = 10
            ax2.set_ylim(min(error_data) - margen,
                         max(error_data) + margen)

    except Exception as e:
        pass  # ignoramos errores de lectura puntual

    return line_setpoint, line_rpm, line_error

# ── ANIMACIÓN ───────────────────────────────────────────
# FuncAnimation llama a actualizar() cada 50ms automáticamente
ani = animation.FuncAnimation(
    fig,
    actualizar,
    interval=50,    # cada 50ms
    blit=True,      # solo redibuja lo que cambia, más eficiente
    cache_frame_data=False
)

print("Gráfica en tiempo real iniciada. Cierra la ventana para parar.")
plt.tight_layout()
plt.show()

# ── CIERRE LIMPIO ───────────────────────────────────────
arduino.close()
print("Conexión cerrada.")
