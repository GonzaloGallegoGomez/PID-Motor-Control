// ── LIBRERÍAS ──────────────────────────────────────────
#include <Encoder.h>

// ── PINES DEL ENCODER ──────────────────────────────────
// El encoder tiene dos señales: canal A y canal B
// Juntas permiten saber dirección y velocidad
#define ENCODER_A 2
#define ENCODER_B 3
// IMPORTANTE: en Arduino Uno, solo los pines 2 y 3
// pueden detectar pulsos rápidos (interrupciones)

// ── OBJETO ENCODER ─────────────────────────────────────
// Creamos un objeto de la librería Encoder
// Le decimos qué pines usar
Encoder motorEncoder(ENCODER_A, ENCODER_B);

// ── VARIABLES ──────────────────────────────────────────
long pulsosActuales = 0;     // cuántos pulsos ha dado el encoder
long pulsoAnteriores = 0;    // pulsos en la medición anterior
float RPM = 0.0;             // velocidad calculada en RPM

// Constante del encoder: pulsos por vuelta completa
// El JGA25-370 tiene 341 pulsos por vuelta (con la reductora)
// Este valor lo tendrás que confirmar con el datasheet de tu motor
const int PULSOS_POR_VUELTA = 341;

// Para medir el tiempo entre mediciones
unsigned long tiempoAnterior = 0;
const int INTERVALO_MS = 100; // medimos cada 100 milisegundos

// ── SETUP ──────────────────────────────────────────────
// En Arduino, setup() se ejecuta UNA sola vez al arrancar
// Es equivalente a la inicialización en Python
void setup() {
  Serial.begin(9600); // abrimos comunicación con el PC a 9600 baudios
                      // esto nos permite ver datos en el monitor serie
}

// ── LOOP ───────────────────────────────────────────────
// loop() se repite indefinidamente, para siempre
// Es equivalente a un while True: en Python
void loop() {

  // Miramos cuánto tiempo ha pasado desde la última medición
  unsigned long tiempoActual = millis(); // millis() da ms desde que arrancó

  if (tiempoActual - tiempoAnterior >= INTERVALO_MS) {

    // Leemos los pulsos acumulados del encoder
    pulsosActuales = motorEncoder.read();

    // Calculamos cuántos pulsos han pasado en este intervalo
    long deltaPulsos = pulsosActuales - pulsoAnteriores;

    // Convertimos pulsos a RPM
    // deltaPulsos / PULSOS_POR_VUELTA = vueltas en el intervalo
    // (1000.0 / INTERVALO_MS) = cuántos intervalos hay en 1 segundo
    // x 60 = pasamos de vueltas/segundo a vueltas/minuto (RPM)
    RPM = (deltaPulsos / (float)PULSOS_POR_VUELTA) * (1000.0 / INTERVALO_MS) * 60.0;

    // Mandamos los datos al PC por el puerto serie
    Serial.print("Pulsos: ");
    Serial.print(pulsosActuales);
    Serial.print("  |  RPM: ");
    Serial.println(RPM);

    // Guardamos valores para la próxima medición
    pulsoAnteriores = pulsosActuales;
    tiempoAnterior = tiempoActual;
  }
}
