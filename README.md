# Field-Oriented Control (FOC) Traction Inverter Firmware

A bare-metal, production-grade Field-Oriented Control (FOC) firmware stack written in ANSI C for 3-phase Permanent Magnet Synchronous Motor (PMSM) automotive traction inverters.

Designed with a modular, layered software architecture targetable to high-performance automotive microcontrollers (TI C2000, STM32G4/F4, Infineon AURIX) and implementing an **E-Gas 3-Level Functional Safety Concept (ISO 26262 ASIL-D)**.

---

## Architecture & System Control Loop

The firmware executes a closed-loop current control pipeline triggered by a $20\text{ kHz}$ ePWM zero-crossing ADC Interrupt Service Routine (ISR).

```
   +---------------------------------------------------------------------------------------+
   |                                 20 kHz ADC ISR Loop                                   |
   |                                                                                       |
   |  [3-Phase Currents] -> ISO 26262 L2 Cross-Check -> Clarke & Park Transforms           |
   |      (I_a, I_b, I_c)       (Debounced Limits)        (I_alpha, I_beta -> I_d, I_q)     |
   |                                                                   |                   |
   |  [ePWM Registers]  <-  ePWM Driver  <- SVPWM Generator <- PI Current Regulators       |
   |   (CMPA, CMPB, CMPC)   (Dead-time)     (Duty Cycles)      (V_d, V_q Output)           |
   +---------------------------------------------------------------------------------------+

```

### Control & Safety Pipeline Steps

1. **Data Acquisition:** Multi-channel ADC reads phase currents ($i_a, i_b, i_c$) sampled at PWM zero-crossings to eliminate high-frequency switching noise.
2. **Level 2 Safety Cross-Checks (`iso26262_fault_monitor`):** Evaluates phase currents and DC bus voltage against safety thresholds using a 3-tick fault debouncer before executing the control loop.
3. **Forward Clarke & Park Transforms:** Converts 3-phase AC quantities into 2-phase stationary ($i_\alpha, i_\beta$) and rotating reference frames ($i_d, i_q$).
4. **PI Current Regulation:** Independent $d$-axis (flux control, setpoint $0\text{ A}$) and $q$-axis (torque control, throttle demand) regulators with anti-windup clamping generate target stator voltages ($v_d, v_q$).
5. **Inverse Park & Space Vector PWM (SVPWM):** Transforms $v_d, v_q$ back to $v_\alpha, v_\beta$ and computes 3-phase duty cycles with common-mode voltage injection ($15.5\%$ higher DC bus utilization).
6. **ePWM Driver & Level 3 Trip Zone:** Converts normalized duty ratios ($0.0\text{ to }1.0$) into timer compare ticks loaded into center-aligned ePWM compare registers (`CMPA`, `CMPB`, `CMPC`) with hardware dead-time insertion. On confirmed faults, directly activates the ePWM Trip Zone to force all outputs LOW within microseconds.

---

## Directory Structure

```text
foc-traction-inverter-firmware/
├── motor/
│   ├── foc_math.h                 # Forward/Inverse Clarke, Park, and SVPWM math routines
│   └── foc_math.c
├── drivers/
│   ├── epwm_driver.h              # Center-aligned ePWM register abstraction, dead-time, & Trip Zone
│   └── epwm_driver.c
├── control/
│   ├── pi_controller.h            # Anti-windup PI controller with voltage saturation limits
│   └── pi_controller.c
├── safety/
│   ├── iso26262_fault_monitor.h    # E-Gas Level 2 safety checks & Level 3 trip logic
│   └── iso26262_fault_monitor.c
├── isr/
│   ├── adc_isr.h                  # 20 kHz Hardware Interrupt Service Routine orchestration
│   └── adc_isr.c
└── main.c                         # Test harness & closed-loop execution simulator

```

---

## Key Technical Features

* **Zero-Dependency Modular C Code:** Written in standard ANSI C (C99/C11) with no vendor locks, enabling easy porting to TI C2000 (CCS), STM32 (CubeIDE), or AUTOSAR Complex Device Drivers (CDD).
* **ISO 26262 Functional Safety (ASIL-D):** Implements Level 2 software safety cross-checks (overcurrent, overvoltage, undervoltage) with configurable fault debouncing to prevent false triggers from transient EMI noise.
* **Level 3 Emergency Shutdown:** Asynchronous hardware trip function (`EPWM_Trip_Zone_Shutdown`) that forces all PWM outputs LOW within microseconds during confirmed faults.
* **Anti-Windup PI Regulators:** Dynamic integral clamping and voltage saturation limiting ($V_{\text{max}} = V_{\text{DC}} / \sqrt{3}$) prevent windup during high-torque step transients.
* **Hardware Dead-Time & Protection:** `epwm_driver` handles dead-time insertion to prevent shoot-through faults in high-voltage H-bridge topologies.

---

## Building and Running

### Prerequisites

* GCC Compiler (`gcc`)
* C standard library (`math.h`)

### Compilation

Compile all modules including the safety driver with optimization flags and math linking (`-lm`):

```bash
gcc -O2 -g main.c motor/foc_math.c drivers/epwm_driver.c control/pi_controller.c safety/iso26262_fault_monitor.c isr/adc_isr.c -o foc_sim -lm

```

### Execution

Run the closed-loop simulator:

```bash
./foc_sim

```

### Expected Sample Output

```text
=========================================================
   CLOSED-LOOP FOC TRACTION INVERTER FIRMWARE SIMULATION  
=========================================================

[ePWM Driver] Configured Center-Aligned PWM @ 20000 Hz | Period: 5000 ticks | Dead-Time: 500 ns
[ISO 26262 Safety] Level 2 Monitor Init | Max Current: 30.0A | DC Bus: 250.0V - 450.0V | Debounce: 3 ticks

--- Starting Closed-Loop Execution (Simulating 20kHz ADC ISR Ticks) ---

[ISR Tick #1]
  -> PWM Compare Registers: CMPA=2845 | CMPB=2154 | CMPC=2154

[ISR Tick #2]
  -> PWM Compare Registers: CMPA=3112 | CMPB=1887 | CMPC=1887

...

--- Testing Safety State Machine ---
[ISO 26262 Safety] CRITICAL FAULT DEBOUNCED (Code: 0x01). Emergency Safe-State Engaged!
[ePWM Driver] HARDWARE TRIP ZONE ACTIVATED: All PWM Outputs Forced LOW!

```

---

## Hardware Integration Targets

| Microcontroller Platform | Timer Peripheral | ADC Trigger Method | Target Application |
| --- | --- | --- | --- |
| **TI C2000 (TMS320F28379D)** | `EPWM1-EPWM3` | `EPWMxSOCA` at Counter Zero | EV Traction Inverters / Solar Inverters |
| **STM32G4 / STM32F4** | `TIM1` / `TIM8` | `TRGO` on Update Event | E-Bikes, Light Electric Vehicles (LEVs) |
| **Infineon AURIX TC3xx** | `GTM_TOM` / `ATOM` | EVADC Trigger Request Unit | Automotive Powertrain VCUs |

---

