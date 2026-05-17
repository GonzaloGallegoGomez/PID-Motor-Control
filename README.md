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
---

### Firmware

| File | Description |
|---|---|
| [encoder_test.ino](firmware/01_encoder_test/encoder_test.ino) | Phase 1 — Reads encoder pulses and calculates real-time RPM using hardware interrupts |
| [motor_driver.ino](firmware/02_motor_driver/motor_driver.ino) | Phase 2 — Controls motor direction and speed via L298N driver using PWM signals |
| [pid_speed_control.ino](firmware/03_pid_speed/pid_speed_control.ino) | Phase 3 — Full PID closed-loop speed controller with anti-windup and CSV serial output |
| [pid_position_control.ino](firmware/04_pid_position/pid_position_control.ino) | Phase 4a — PID angular position controller with dead zone and automatic direction control |
| [cruise_control.ino](firmware/05_cruise_control/cruise_control.ino) | Phase 4b — Cruise control mode with real-time serial commands to activate, deactivate and adjust setpoint |

### Simulation

| File | Description |
|---|---|
| [pid_simulation.m](simulation/matlab/pid_simulation.m) | MATLAB script — derives motor transfer function from physical equations and compares P, PI and PID response |
| [plot_response.py](simulation/python/plot_response.py) | Python — generic real-time serial plotter for PID speed controller |
| [cruise_control_plotter.py](simulation/python/cruise_control_plotter.py) | Python — cruise control plotter with active zone visualization and setpoint change markers |

### Hardware

| File | Description |
|---|---|
| [BOM.md](hardware/BOM.md) | Bill of materials with component list, prices and pin connection map |
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

### MATLAB Simulation 

| Controller | Rise time | Settling time | Overshoot | Steady-state error |
|---|---|---|---|---|
| P only | 0.054 s | 0.097 s | 0% | ~37% |
| PI | 0.289 s | 0.678 s | 0% | 0% |
| PID | 0.322 s | 0.644 s | 0% | 0% |

### MATLAB Simulation

![P vs PI vs PID Response](https://raw.githubusercontent.com/GonzaloGallegoGomez/PID-Motor-Control/main/simulation/matlab/comparison_P_PI_PID.png)

### Simulation results

The following graph shows the step response comparison between P, PI and PID controllers
simulated in MATLAB using the motor's physical transfer function derived from first principles:

![P vs PI vs PID Response](https://raw.githubusercontent.com/GonzaloGallegoGomez/PID-Motor-Control/main/simulation/matlab/comparison_P_PI_PID.png)

| Controller | Rise time | Settling time | Overshoot | Steady-state error |
|---|---|---|---|---|
| P only | 0.054 s | 0.097 s | 0% | ~37% |
| PI | 0.289 s | 0.678 s | 0% | 0% |
| PID | 0.322 s | 0.644 s | 0% | 0% |

> **Key insight:** The P controller reaches steady state fastest but never eliminates the
> steady-state error. The PI controller eliminates the error but is slower. The PID controller
> combines both — faster settling than PI with zero steady-state error.

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
