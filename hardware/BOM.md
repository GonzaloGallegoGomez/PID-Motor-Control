# Bill of Materials (BOM)
## PID Motor Control System — Hardware Components

> All components tested and selected for compatibility with Arduino Uno and 12V DC motor control.

---

## Components List

| # | Component | Model | Quantity | Unit Price (€) | Total (€) | Purchase Link |
|---|---|---|---|---|---|---|
| 1 | Microcontroller | Arduino Uno R3 | 1 | ~12.00 | ~12.00 | Amazon.es |
| 2 | DC Motor with encoder | JGA25-370 12V | 1 | ~14.00 | ~14.00 | Amazon.es |
| 3 | Motor driver | L298N | 1 | ~5.00 | ~5.00 | Amazon.es |
| 4 | Breadboard | 830 points | 1 | ~4.00 | ~4.00 | Amazon.es |
| 5 | Jumper wires | M-M and M-F kit | 1 | ~5.00 | ~5.00 | Amazon.es |
| 6 | Potentiometer | 10kΩ | 1 | ~2.00 | ~2.00 | Amazon.es |
| 7 | Power supply | 12V ≥ 1A | 1 | ✅ Already owned | — | — |

**Estimated total: ~42€**

---

## Pin Connection Map

### Encoder → Arduino
| Encoder Pin | Arduino Pin | Description |
|---|---|---|
| VCC | 5V | Power supply |
| GND | GND | Common ground |
| Channel A | Pin 2 | Interrupt pin |
| Channel B | Pin 3 | Interrupt pin |

### Arduino → L298N
| Arduino Pin | L298N Pin | Description |
|---|---|---|
| Pin 5 (PWM) | ENA | Speed control via PWM |
| Pin 7 | IN1 | Direction control 1 |
| Pin 8 | IN2 | Direction control 2 |
| GND | GND | Common ground |

### Power Supply → L298N
| Power Supply | L298N Pin | Description |
|---|---|---|
| 12V + | VCC | Motor power input |
| GND | GND | Common ground |

---

## Important Notes

- **Power supply**: Must be 12V and at least 1A. Do NOT power the motor directly from the Arduino 5V pin — this will damage the board.
- **Encoder pins 2 & 3**: These are the only hardware interrupt pins on Arduino Uno. Required for accurate high-speed pulse counting.
- **L298N logic vs motor power**: The L298N takes 5V logic from Arduino and 12V motor power from the external supply separately. Both GND must be connected together (common ground).
- **Encoder PPR**: The JGA25-370 outputs 341 pulses per revolution (after gearbox). Verify with your specific unit's datasheet as this may vary.

---

## Tools Required

| Tool | Required for |
|---|---|
| USB-A to USB-B cable | Arduino programming |
| Multimeter | Circuit verification and debugging |
| Small screwdriver | L298N terminal blocks |

---

*Last updated: May 2026*
