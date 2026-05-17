// ══════════════════════════════════════════════════════
//  Cruise Control — Fase 4b
//  Gonzalo Gallego Gomez — EEBE UPC
//
//  Simula un sistema de cruise control sobre un motor DC:
//  - Activa y desactiva el control con comandos serie
//  - Sube y baja la velocidad objetivo gradualmente
//  - Mantiene RPM constantes ante perturbaciones externas
//
//  Comandos por Serial Monitor (sin comillas):
//  'a' → activar cruise control
//  'd' → desactivar (motor libre)
//  '+' → subir velocidad +10 RPM
//  '-' → bajar velocidad -10 RPM
//  'r' → reset a velocidad inicial
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
const float PULSOS_POR_VUELTA = 341.0;

// ── CONFIGURACIÓN DEL CRUISE CONTROL ──────────────────
float setpoint        = 100.0;  // RPM iniciales al activar
const float RPM_MIN   = 30.0;   // RPM mínimas permitidas
const float RPM_MAX   = 200.0;  // RPM máximas permitidas
const float RPM_PASO  = 10.0;   // cuánto sube/baja cada vez
                                 // que pulsas + o -

// ── ESTADO DEL SISTEMA ─────────────────────────────────
// bool es un tipo de dato que solo puede ser true o false
// equivalente a un booleano en Python
bool cruiseActivo = false;  // empieza desactivado

// ── PARÁMETROS PID ─────────────────────────────────────
float Kp = 1.5;
float Ki = 0.8;
float Kd = 0.05;

// ── VARIABLES DEL CONTROLADOR PID ──────────────────────
float error         = 0;
float errorAnterior = 0;
float integrador    = 0;
float salidaPID     = 0;
float RPM           = 0;

// ── VARIABLES DE TIEMPO Y ENCODER ──────────────────────
long pulsosActuales   = 0;
long pulsosAnteriores = 0;
unsigned long tiempoAnterior = 0;
const int INTERVALO_MS = 100;

// ── SETUP ──────────────────────────────────────────────
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  Serial.begin(9600);
  Serial.println("═══════════════════════════════════════");
  Serial.println("  CRUISE CONTROL — PID Motor Control");
  Serial.println("═══════════════════════════════════════");
  Serial.println("Comandos:");
  Serial.println("  'a' → Activar cruise control");
  Serial.println("  'd' → Desactivar");
  Serial.println("  '+' → Subir velocidad +10 RPM");
  Serial.println("  '-' → Bajar velocidad -10 RPM");
  Serial.println("  'r' → Reset a 100 RPM");
  Serial.println("═══════════════════════════════════════");
  Serial.println("estado,setpoint,RPM,salidaPID");
}

// ── FUNCIONES DE MOTOR ─────────────────────────────────
void adelante(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);
}

void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// ── FUNCIÓN: LEER COMANDOS SERIE ───────────────────────
// Comprueba si el usuario ha enviado algún comando
// desde el Serial Monitor del PC
void leerComandos() {

  // Serial.available() devuelve cuántos bytes hay
  // pendientes de leer en el buffer serie
  // Es como comprobar si hay mensajes nuevos
  if (Serial.available() > 0) {

    // Leemos un carácter
    char comando = Serial.read();

    switch (comando) {

      case 'a':  // Activar cruise control
        cruiseActivo = true;
        integrador = 0;    // reseteamos el integrador al activar
        errorAnterior = 0; // para evitar un pico derivativo inicial
        Serial.print("CRUISE ACTIVADO — Setpoint: ");
        Serial.print(setpoint);
        Serial.println(" RPM");
        break;

      case 'd':  // Desactivar cruise control
        cruiseActivo = false;
        parar();
        Serial.println("CRUISE DESACTIVADO — Motor libre");
        break;

      case '+':  // Subir velocidad
        if (cruiseActivo) {
          setpoint = constrain(setpoint + RPM_PASO, RPM_MIN, RPM_MAX);
          Serial.print("Setpoint subido a: ");
          Serial.print(setpoint);
          Serial.println(" RPM");
        } else {
          Serial.println("Activa el cruise primero con 'a'");
        }
        break;

      case '-':  // Bajar velocidad
        if (cruiseActivo) {
          setpoint = constrain(setpoint - RPM_PASO, RPM_MIN, RPM_MAX);
          Serial.print("Setpoint bajado a: ");
          Serial.print(setpoint);
          Serial.println(" RPM");
        } else {
          Serial.println("Activa el cruise primero con 'a'");
        }
        break;

      case 'r':  // Reset
        setpoint = 100.0;
        integrador = 0;
        Serial.println("Reset a 100 RPM");
        break;
    }
  }
}

// ── LOOP ───────────────────────────────────────────────
void loop() {

  // Siempre comprobamos si hay comandos nuevos
  // independientemente del intervalo PID
  leerComandos();

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= INTERVALO_MS) {

    // ── 1. LEER ENCODER Y CALCULAR RPM ─────────────────
    pulsosActuales = motorEncoder.read();
    long deltaPulsos = pulsosActuales - pulsosAnteriores;

    RPM = (deltaPulsos / PULSOS_POR_VUELTA)
          * (1000.0 / INTERVALO_MS)
          * 60.0;
    RPM = abs(RPM);

    // ── 2. LÓGICA CRUISE CONTROL ────────────────────────
    if (cruiseActivo) {

      // ── PID activo ─────────────────────────────────
      error = setpoint - RPM;

      float termP = Kp * error;

      integrador += error * (INTERVALO_MS / 1000.0);
      integrador = constrain(integrador, -100, 100);
      float termI = Ki * integrador;

      float derivada = (error - errorAnterior) / (INTERVALO_MS / 1000.0);
      float termD = Kd * derivada;

      salidaPID = termP + termI + termD;
      salidaPID = constrain(salidaPID, 0, 255);

      adelante((int)salidaPID);

      // Enviamos datos en formato CSV para Python plotter
      Serial.print("ON,");
      Serial.print(setpoint);  Serial.print(",");
      Serial.print(RPM);       Serial.print(",");
      Serial.println(salidaPID);

    } else {

      // ── Cruise desactivado: motor parado ───────────
      salidaPID = 0;

      Serial.print("OFF,");
      Serial.print(setpoint);  Serial.print(",");
      Serial.print(RPM);       Serial.print(",");
      Serial.println(0);
    }

    // ── 3. GUARDAR VALORES ──────────────────────────────
    errorAnterior    = error;
    pulsosAnteriores = pulsosActuales;
    tiempoAnterior   = tiempoActual;
  }
}
