# ══════════════════════════════════════════════════════
#  Cruise Control — Real-time Plotter
#  Gonzalo Gallego Gomez — EEBE UPC
#
#  Lee datos del Arduino por puerto serie y muestra:
#  - RPM actuales vs setpoint
#  - Estado del cruise (activo/inactivo)
#  - Salida PWM del controlador PID
#  - Marcadores de cambio de setpoint
#
#  Formato CSV esperado del Arduino:
#  estado,setpoint,RPM,salidaPID
#  ON,100,98.3,142
#  OFF,100,0,0
# ══════════════════════════════════════════════════════

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from collections import deque
import time

# ── CONFIGURACIÓN ───────────────────────────────────────
PUERTO     = 'COM3'   # cambia por tu puerto real
BAUDIOS    = 9600
MAX_DATOS  = 150      # ventana de 150 puntos × 100ms = 15 segundos

# ── BUFFERS DE DATOS ────────────────────────────────────
setpoint_data = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
rpm_data      = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
pwm_data      = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)
estado_data   = deque(['OFF'] * MAX_DATOS, maxlen=MAX_DATOS)
tiempo_data   = deque([0] * MAX_DATOS, maxlen=MAX_DATOS)

# Para detectar cambios de setpoint y marcarlos
setpoint_anterior = 100.0
cambios_setpoint  = []  # lista de tiempos donde cambió el setpoint

tiempo_inicio = time.time()

# ── CONEXIÓN CON EL ARDUINO ─────────────────────────────
try:
    arduino = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    time.sleep(2)
    print(f"Conectado al Arduino en {PUERTO}")
    print("Comandos: 'a' activar, 'd' desactivar, '+' subir, '-' bajar, 'r' reset")
    print("Escríbelos en el Serial Monitor del Arduino IDE")
except Exception as e:
    print(f"Error al conectar: {e}")
    exit()

# ── CONFIGURACIÓN DE LA GRÁFICA ─────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle('Cruise Control — PID Motor Control\nGonzalo Gallego Gomez | EEBE UPC',
             fontsize=13, fontweight='bold')

# ── GRÁFICA 1: RPM vs Setpoint ──────────────────────────
ax1.set_ylabel('Speed (RPM)')
ax1.set_xlabel('Time (s)')
ax1.set_title('Motor Speed vs Setpoint')
ax1.grid(True, alpha=0.3)
ax1.set_ylim(-10, 250)

line_setpoint, = ax1.plot([], [], 'r--', linewidth=2,
                           label='Setpoint (RPM)')
line_rpm,      = ax1.plot([], [], 'b-',  linewidth=2,
                           label='Actual RPM')

# Leyenda con parche para zona activa
parche_activo = mpatches.Patch(color='green', alpha=0.15,
                                label='Cruise active')
ax1.legend(handles=[line_setpoint, line_rpm, parche_activo],
           loc='upper right')

# ── GRÁFICA 2: Salida PWM ───────────────────────────────
ax2.set_ylabel('PWM Output (0-255)')
ax2.set_xlabel('Time (s)')
ax2.set_title('PID Controller Output')
ax2.grid(True, alpha=0.3)
ax2.set_ylim(-10, 270)
ax2.axhline(y=255, color='r', linewidth=0.8,
            linestyle=':', label='Max PWM (255)')
ax2.axhline(y=0,   color='k', linewidth=0.5)

line_pwm, = ax2.plot([], [], 'orange', linewidth=2,
                      label='PWM output')
ax2.legend(loc='upper right')

# ── FUNCIÓN DE ACTUALIZACIÓN ────────────────────────────
def actualizar(frame):
    global setpoint_anterior

    try:
        linea = arduino.readline().decode('utf-8').strip()

        # Ignoramos cabeceras, líneas de texto y vacías
        if ',' not in linea or 'estado' in linea or 'CRUISE' in linea \
           or 'Setpoint' in linea or 'Reset' in linea \
           or 'Activa' in linea or 'Comandos' in linea:
            return line_setpoint, line_rpm, line_pwm

        valores = linea.split(',')
        if len(valores) != 4:
            return line_setpoint, line_rpm, line_pwm

        estado = valores[0].strip()           # ON o OFF
        sp     = float(valores[1])            # setpoint RPM
        rpm    = float(valores[2])            # RPM actuales
        pwm    = float(valores[3])            # salida PID

        t = time.time() - tiempo_inicio

        # Detectamos cambio de setpoint y lo guardamos
        if abs(sp - setpoint_anterior) > 1.0:
            cambios_setpoint.append(t)
            setpoint_anterior = sp

        # Añadimos datos a los buffers
        setpoint_data.append(sp)
        rpm_data.append(rpm)
        pwm_data.append(pwm)
        estado_data.append(estado)
        tiempo_data.append(t)

        t_list  = list(tiempo_data)
        sp_list = list(setpoint_data)
        rpm_list = list(rpm_data)
        pwm_list = list(pwm_data)
        est_list = list(estado_data)

        # Actualizamos las líneas
        line_setpoint.set_data(t_list, sp_list)
        line_rpm.set_data(t_list, rpm_list)
        line_pwm.set_data(t_list, pwm_list)

        # Ajustamos eje X deslizante
        x_min = max(0, t - 15)
        x_max = t + 1
        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)

        # ── Sombreado verde cuando cruise está activo ───
        # Limpiamos sombreados anteriores para redibujar
        for coleccion in ax1.collections:
            coleccion.remove()

        # Buscamos segmentos donde el cruise está activo
        inicio_segmento = None
        for i, est in enumerate(est_list):
            if est == 'ON' and inicio_segmento is None:
                inicio_segmento = t_list[i]
            elif est == 'OFF' and inicio_segmento is not None:
                ax1.axvspan(inicio_segmento, t_list[i],
                           alpha=0.15, color='green')
                inicio_segmento = None

        # Cerramos el último segmento si sigue activo
        if inicio_segmento is not None:
            ax1.axvspan(inicio_segmento, t,
                       alpha=0.15, color='green')

        # ── Líneas verticales en cambios de setpoint ────
        for t_cambio in cambios_setpoint:
            if x_min <= t_cambio <= x_max:
                ax1.axvline(x=t_cambio, color='red',
                           linewidth=1, linestyle=':',
                           alpha=0.7)

    except Exception:
        pass

    return line_setpoint, line_rpm, line_pwm

# ── ANIMACIÓN ───────────────────────────────────────────
ani = animation.FuncAnimation(
    fig,
    actualizar,
    interval=50,
    blit=False,        # False porque redibujamos los axvspan
    cache_frame_data=False
)

print("Plotter iniciado. Cierra la ventana para parar.")
plt.tight_layout()
plt.show()

# ── CIERRE ──────────────────────────────────────────────
arduino.close()
print("Conexión cerrada.")
