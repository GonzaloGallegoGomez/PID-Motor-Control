# PID Motor Control System 🎛️

> PID controller implemented on Arduino to regulate speed (RPM) and angular position of a DC motor with encoder. Built as a portfolio project during my 2nd year of Mechanical Engineering at EEBE (UPC).

---

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware](#hardware)
- [Software & Dependencies](#software--dependencies)
- [Repository Structure](#repository-structure)
- [Theory: How PID Works](#theory-how-pid-works)
- [Results](#results)
- [Roadmap](#roadmap)
- [Author](#author)

---

## Overview

This project implements a closed-loop PID control system on an Arduino Uno to control a 12V DC motor with a quadrature encoder. The system reads real-time feedback from the encoder, computes the error signal, and applies proportional, integral, and derivative corrections to drive the motor to a desired setpoint.

Three control modes are demonstrated:
- **Constant RPM** — maintain a target speed regardless of load
- **Angular position** — rotate the motor shaft to an exact angle
- **Cruise control** — simulate automotive cruise control behavior

---

## Features

- ✅ Real-time encoder reading via hardware interrupts (pins 2 & 3)
- ✅ Tunable Kp, Ki, Kd parameters
- ✅ Serial output for live monitoring and data logging
- ✅ Python script for plotting system response
- ✅ MATLAB/Simulink simulation for theoretical validation
- ✅ Comparison of P, PI and PID response curves
- 🔲 OLED display for live RPM readout *(coming soon)*
- 🔲 Potentiometer setpoint control *(coming soon)*

---

## Hardware

### Components

| Component | Model | Quantity |
|---|---|---|
| Microcontroller | Arduino Uno R3 | 1 |
| DC Motor with encoder | JGA25-370 12V | 1 |
| Motor driver | L298N | 1 |
| Breadboard | 830 points | 1 |
| Jumper wires | M-M and M-F | 1 kit |
| Potentiometer | 10kΩ | 1 |
| Power supply | 12V ≥ 1A | 1 |

### Circuit Diagram

> 🔧 *Circuit diagram coming soon — will be added in `/hardware/circuit_diagram.fzz`*

### Pin Connections

| Arduino Pin | Connected to |
|---|---|
| 2 | Encoder channel A |
| 3 | Encoder channel B |
| 5 (PWM) | L298N ENA |
| 6 (PWM) | L298N ENB |
| 7 | L298N IN1 |
| 8 | L298N IN2 |
| 5V | Encoder VCC |
| GND | Common ground |

---

## Software & Dependencies

### Arduino Libraries
| Library | Author | Install via |
|---|---|---|
| `PID_v1` | Brett Beauregard | Arduino Library Manager |
| `Encoder` | Paul Stoffregen | Arduino Library Manager |

### Python Libraries
```bash
pip install matplotlib numpy pyserial
```

### Tools
- [Arduino IDE 2.x](https://www.arduino.cc/en/software)
- [MATLAB R2024b](https://mathworks.com) + Simulink + Control System Toolbox
- [Fritzing](https://fritzing.org) — circuit diagrams
- Python 3.x

---

## Repository Structure

```
PID-Motor-Control/
│
├── README.md
│
├── docs/                        # Theory, memory and circuit diagrams
│   ├── pid_theory.md
│   └── circuit_diagram.pdf
│
├── hardware/                    # Fritzing files and BOM
│   ├── circuit.fzz
│   └── BOM.md
│
├── firmware/                    # Arduino sketches
│   ├── 01_encoder_test/
│   │   └── encoder_test.ino
│   ├── 02_motor_driver/
│   │   └── motor_driver.ino
│   ├── 03_pid_speed/
│   │   └── pid_speed_control.ino
│   ├── 04_pid_position/
│   │   └── pid_position_control.ino
│   └── 05_cruise_control/
│       └── cruise_control.ino
│
├── simulation/                  # MATLAB/Simulink models and Python scripts
│   ├── matlab/
│   │   └── pid_simulation.slx
│   └── python/
│       └── plot_response.py
│
├── results/                     # Graphs and response curves
│   ├── p_response.png
│   ├── pi_response.png
│   └── pid_response.png
│
└── media/                       # Photos, videos and GIFs
    ├── demo_rpm.gif
    ├── demo_position.gif
    └── setup_photo.jpg
```

---

## Theory: How PID Works

A PID controller continuously calculates an **error** value as the difference between a desired setpoint and a measured process variable, and applies a correction based on three terms:

$$u(t) = K_p \cdot e(t) + K_i \int e(t)\, dt + K_d \frac{de(t)}{dt}$$

| Term | Symbol | Effect |
|---|---|---|
| Proportional | Kp | Reacts to current error. Higher Kp = faster response but more overshoot |
| Integral | Ki | Eliminates steady-state error by accumulating past errors |
| Derivative | Kd | Anticipates future error by reacting to rate of change. Reduces overshoot |

### Tuning strategy used
Parameters were tuned using the **Ziegler-Nichols** method: Kp was increased until the system oscillated at a constant amplitude (critical gain Ku), then Ki and Kd were derived from the oscillation period Tu.

---

## Results

> 📊 *Results and response graphs will be added here once hardware testing is complete.*

### MATLAB Simulation *(coming soon)*

| Controller | Rise time | Overshoot | Steady-state error |
|---|---|---|---|
| P only | — | — | — |
| PI | — | — | — |
| PID | — | — | — |

### Hardware Demo *(coming soon)*

| Demo | Status |
|---|---|
| Constant RPM hold | 🔲 Pending |
| Angular position control | 🔲 Pending |
| Cruise control mode | 🔲 Pending |

---

## Roadmap

- [x] Repository setup and documentation structure
- [x] Arduino encoder reading (Phase 1)
- [ ] Motor driver control (Phase 2)
- [ ] Full PID controller implementation (Phase 3)
- [ ] Three control mode demos (Phase 4)
- [ ] MATLAB/Simulink simulation
- [ ] Python response plotting script
- [ ] Hardware assembly and testing
- [ ] Final results and media

---

## Author

**Gonzalo Gallego Gomez**
2nd year Mechanical Engineering student — EEBE, Universitat Politècnica de Catalunya (UPC)

[![GitHub](https://img.shields.io/badge/GitHub-GonzaloGallegoGomez-181717?logo=github)](https://github.com/GonzaloGallegoGomez)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0A66C2?logo=linkedin)](https://www.linkedin.com/in/gonzalo-gallego-g%C3%B3mez-1263a2350/)

---

*This project is part of a personal engineering portfolio. Feel free to use it as reference — a ⭐ is always appreciated!*
