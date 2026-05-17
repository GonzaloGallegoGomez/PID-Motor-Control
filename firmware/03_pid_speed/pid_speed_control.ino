// ══════════════════════════════════════════════════════
//  PID Speed Controller — Fase 3
//  Gonzalo Gallego Gomez — EEBE UPC
//  Controla las RPM de un motor DC con encoder
//  usando un controlador PID implementado manualmente
// ══════════════════════════════════════════════════════

// ── LIBRERÍAS ──────────────────────────────────────────
#include <Encoder.h>
// Nota: implementamos el PID manualmente en lugar de usar
// la librería PID_v1, para entender cada línea del algoritmo

// ── PINES ──────────────────────────────────────────────
#define ENCODER_A 2
#define ENCODER_B 3
#define ENA       5
#define IN1       7
#define IN2       8

// ── OBJETO ENCODER ─────────────────────────────────────
Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ── PARÁMETROS DEL MOTOR ───────────────────────────────
const int PULSOS_POR_VUELTA = 341;  // JGA25-370

// ── SETPOINT ───────────────────────────────────────────
// Velocidad objetivo en RPM
// Este es el valor que queremos mantener constante
float setpoint = 100.0;  // 100 RPM

// ── PARÁMETROS PID ─────────────────────────────────────
// Estos valores son el resultado del tuning
// Los ajustaremos cuando tengamos el hardware real
float Kp = 1.5;
float Ki = 0.8;
float Kd = 0.05;

// ── VARIABLES DEL CONTROLADOR PID ──────────────────────
float error          = 0;  // diferencia entre setpoint y RPM actual
float errorAnterior  = 0;  // error en la iteración anterior (para D)
float integrador     = 0;  // acumulador del término integral
float derivada       = 0;  // término derivativo
float salidaPID      = 0;  // resultado del cálculo PID (0-255)

// ── VARIABLES DE TIEMPO Y ENCODER ──────────────────────
long pulsosActuales  = 0;
long pulsosAnteriores = 0;
float RPM            = 0;
unsigned long tiempoAnterior = 0;
const int INTERVALO_MS = 100;  // calculamos PID cada 100ms

// ── SETUP ──────────────────────────────────────────────
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Arrancamos siempre hacia adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  Serial.begin(9600);
  Serial.println("setpoint,RPM,salidaPID,error");
  // Imprimimos cabecera CSV para poder graficar con Python
}

// ── LOOP ───────────────────────────────────────────────
void loop() {

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= INTERVALO_MS) {

    // ── 1. LEER ENCODER Y CALCULAR RPM ─────────────────
    pulsosActuales = motorEncoder.read();
    long deltaPulsos = pulsosActuales - pulsosAnteriores;

    // Convertimos pulsos a RPM
    RPM = (deltaPulsos / (float)PULSOS_POR_VUELTA)
          * (1000.0 / INTERVALO_MS)
          * 60.0;

    // Tomamos valor absoluto — el PID controla magnitud,
    // la dirección la fijamos con IN1/IN2
    RPM = abs(RPM);

    // ── 2. CALCULAR ERROR ───────────────────────────────
    // Error = lo que queremos - lo que tenemos
    // Si error > 0 el motor va lento → hay que acelerar
    // Si error < 0 el motor va rápido → hay que frenar
    error = setpoint - RPM;

    // ── 3. TÉRMINO PROPORCIONAL ─────────────────────────
    // Reacciona al error actual
    // Mayor error → mayor corrección
    float termP = Kp * error;

    // ── 4. TÉRMINO INTEGRAL ─────────────────────────────
    // Acumula el error a lo largo del tiempo
    // Elimina el error en estado estacionario
    integrador += error * (INTERVALO_MS / 1000.0);

    // Anti-windup: limitamos el integrador para evitar
    // que se acumule demasiado y cause inestabilidad
    integrador = constrain(integrador, -100, 100);

    float termI = Ki * integrador;

    // ── 5. TÉRMINO DERIVATIVO ───────────────────────────
    // Reacciona a la velocidad de cambio del error
    // Anticipa el comportamiento futuro y reduce overshoot
    derivada = (error - errorAnterior) / (INTERVALO_MS / 1000.0);
    float termD = Kd * derivada;

    // ── 6. SUMA PID ─────────────────────────────────────
    salidaPID = termP + termI + termD;

    // Limitamos la salida entre 0 y 255 (rango del PWM)
    // constrain() es equivalente a clip() en numpy
    salidaPID = constrain(salidaPID, 0, 255);

    // ── 7. APLICAR AL MOTOR ─────────────────────────────
    analogWrite(ENA, (int)salidaPID);

    // ── 8. ENVIAR DATOS AL PC ───────────────────────────
    // Formato CSV para poder leerlo con Python
    Serial.print(setpoint);  Serial.print(",");
    Serial.print(RPM);        Serial.print(",");
    Serial.print(salidaPID);  Serial.print(",");
    Serial.println(error);

    // ── 9. GUARDAR VALORES PARA SIGUIENTE ITERACIÓN ────
    errorAnterior  = error;
    pulsosAnteriores = pulsosActuales;
    tiempoAnterior = tiempoActual;
  }
}
