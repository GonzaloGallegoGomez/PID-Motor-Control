// ══════════════════════════════════════════════════════
//  PID Position Controller — Fase 4a
//  Gonzalo Gallego Gomez — EEBE UPC
//  Controla la posición angular de un motor DC con
//  encoder usando un controlador PID en bucle cerrado
// ══════════════════════════════════════════════════════

// ── LIBRERÍAS ──────────────────────────────────────────
#include <Encoder.h>

// ── PINES ──────────────────────────────────────────────
#define ENCODER_A 2
#define ENCODER_B 3
#define ENA       5
#define IN1       7
#define IN2       8

// ── OBJETO ENCODER ─────────────────────────────────────
Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ── PARÁMETROS DEL MOTOR ───────────────────────────────
const float PULSOS_POR_VUELTA = 341.0;  // JGA25-370

// ── SETPOINT DE POSICIÓN ───────────────────────────────
// Posición objetivo en grados
// Puedes cambiar este valor para probar distintos ángulos
float setpointGrados = 180.0;  // queremos girar 180°

// Convertimos grados a pulsos para comparar con el encoder
// El PID trabaja en pulsos internamente — más preciso
float setpointPulsos = (setpointGrados / 360.0) * PULSOS_POR_VUELTA;

// ── PARÁMETROS PID ─────────────────────────────────────
// Para posición los valores son distintos que para velocidad
// El sistema es diferente — ajustar con el hardware real
float Kp = 2.0;
float Ki = 0.1;
float Kd = 0.5;   // Kd más alto que en velocidad para frenar
                   // antes de llegar al setpoint y no pasarse

// ── VARIABLES DEL CONTROLADOR PID ──────────────────────
float error         = 0;
float errorAnterior = 0;
float integrador    = 0;
float salidaPID     = 0;

// ── VARIABLES DE TIEMPO ────────────────────────────────
unsigned long tiempoAnterior = 0;
const int INTERVALO_MS = 20;  // 20ms para posición
                               // más rápido que velocidad
                               // necesitamos más precisión

// ── ZONA MUERTA ────────────────────────────────────────
// Si el error es menor que esto consideramos que hemos
// llegado al destino y paramos el motor
// Evita que el motor vibre alrededor del setpoint
const float ZONA_MUERTA_PULSOS = 2.0;  // ±2 pulsos ≈ ±2°

// ── VELOCIDAD MÍNIMA ───────────────────────────────────
// PWM mínimo para que el motor pueda moverse
// Por debajo de este valor el motor no tiene suficiente
// par para vencer la fricción estática
const int PWM_MINIMO = 50;

// ── SETUP ──────────────────────────────────────────────
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Arrancamos con el motor parado
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Reseteamos el encoder a 0 — posición inicial
  motorEncoder.write(0);

  Serial.begin(9600);
  Serial.println("setpoint_grados,posicion_grados,salida_pid,error_grados");
}

// ── FUNCIONES DE DIRECCIÓN ─────────────────────────────
void adelante(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);
}

void atras(int velocidad) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, velocidad);
}

void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// ── LOOP ───────────────────────────────────────────────
void loop() {

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= INTERVALO_MS) {

    // ── 1. LEER POSICIÓN ACTUAL ─────────────────────────
    // En posición leemos pulsos acumulados directamente
    // No calculamos RPM — nos interesa dónde está, no
    // cómo de rápido va
    float pulsosActuales = (float)motorEncoder.read();

    // Convertimos pulsos a grados para el Serial Monitor
    float gradosActuales = (pulsosActuales / PULSOS_POR_VUELTA) * 360.0;

    // ── 2. CALCULAR ERROR ───────────────────────────────
    // Error en pulsos: diferencia entre donde queremos
    // estar y donde estamos
    error = setpointPulsos - pulsosActuales;

    // ── 3. ZONA MUERTA ──────────────────────────────────
    // Si estamos suficientemente cerca del objetivo
    // paramos el motor y salimos del cálculo PID
    if (abs(error) <= ZONA_MUERTA_PULSOS) {
      parar();

      // Seguimos enviando datos para ver que estamos estables
      Serial.print(setpointGrados);  Serial.print(",");
      Serial.print(gradosActuales);  Serial.print(",");
      Serial.print(0);               Serial.print(",");
      Serial.println((error / PULSOS_POR_VUELTA) * 360.0);
      return;  // saltamos el resto del loop
               // return dentro de loop() pasa a la siguiente
               // iteración, no termina el programa
    }

    // ── 4. TÉRMINO PROPORCIONAL ─────────────────────────
    float termP = Kp * error;

    // ── 5. TÉRMINO INTEGRAL ─────────────────────────────
    integrador += error * (INTERVALO_MS / 1000.0);
    integrador = constrain(integrador, -200, 200);  // anti-windup
    float termI = Ki * integrador;

    // ── 6. TÉRMINO DERIVATIVO ───────────────────────────
    // En posición el término D es crucial — frena el motor
    // antes de llegar al destino para no pasarse
    float derivada = (error - errorAnterior) / (INTERVALO_MS / 1000.0);
    float termD = Kd * derivada;

    // ── 7. SUMA PID ─────────────────────────────────────
    salidaPID = termP + termI + termD;

    // ── 8. APLICAR DIRECCIÓN Y VELOCIDAD ───────────────
    // A diferencia del control de velocidad, aquí el signo
    // de salidaPID nos dice la dirección de movimiento
    // Positivo → hay que ir adelante
    // Negativo → hay que ir atrás
    int pwm = (int)constrain(abs(salidaPID), 0, 255);

    // Aplicamos velocidad mínima para vencer la fricción
    if (pwm > 0 && pwm < PWM_MINIMO) {
      pwm = PWM_MINIMO;
    }

    if (salidaPID > 0) {
      adelante(pwm);
    } else {
      atras(pwm);
    }

    // ── 9. ENVIAR DATOS AL PC ───────────────────────────
    float errorGrados = (error / PULSOS_POR_VUELTA) * 360.0;

    Serial.print(setpointGrados);  Serial.print(",");
    Serial.print(gradosActuales);  Serial.print(",");
    Serial.print(salidaPID);       Serial.print(",");
    Serial.println(errorGrados);

    // ── 10. GUARDAR VALORES ─────────────────────────────
    errorAnterior = error;
    tiempoAnterior = tiempoActual;
  }
}
