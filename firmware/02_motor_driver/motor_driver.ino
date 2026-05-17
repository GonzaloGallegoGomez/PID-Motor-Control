// ── PINES DEL DRIVER L298N ─────────────────────────────
// ENA: controla la velocidad mediante PWM
// IN1, IN2: controlan la dirección de giro
#define ENA 5    // pin PWM (~) para velocidad
#define IN1 7    // dirección 1
#define IN2 8    // dirección 2

// ── SETUP ──────────────────────────────────────────────
void setup() {
  // Declaramos los tres pines como salidas
  // En Python sería equivalente a configurar un GPIO
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Motor driver listo");
}

// ── FUNCIONES DE CONTROL ───────────────────────────────
// En Arduino podemos definir funciones fuera del loop()
// igual que en Python definimos funciones con def

// Girar hacia adelante a una velocidad dada (0-255)
void adelante(int velocidad) {
  digitalWrite(IN1, HIGH);  // IN1 = 1, IN2 = 0 → adelante
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velocidad);  // PWM: 0-255
}

// Girar hacia atrás
void atras(int velocidad) {
  digitalWrite(IN1, LOW);   // IN1 = 0, IN2 = 1 → atrás
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, velocidad);
}

// Parar el motor
void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// ── LOOP ───────────────────────────────────────────────
void loop() {

  // Test 1: adelante al 50% de potencia durante 2 segundos
  Serial.println("Adelante 50%");
  adelante(127);       // 127/255 ≈ 50%
  delay(2000);         // esperamos 2000ms = 2 segundos
                       // delay() es como time.sleep() en Python

  // Test 2: parar 1 segundo
  Serial.println("Parando...");
  parar();
  delay(1000);

  // Test 3: adelante al 100% durante 2 segundos
  Serial.println("Adelante 100%");
  adelante(255);
  delay(2000);

  // Test 4: parar 1 segundo
  parar();
  delay(1000);

  // Test 5: atrás al 75% durante 2 segundos
  Serial.println("Atras 75%");
  atras(191);          // 191/255 ≈ 75%
  delay(2000);

  // Test 6: parar y repetir el ciclo
  Serial.println("Parando...");
  parar();
  delay(1000);
}
