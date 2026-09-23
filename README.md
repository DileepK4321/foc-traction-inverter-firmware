# Field-Oriented Control (FOC) Traction Inverter Firmware

A bare-metal, production-grade Field-Oriented Control (FOC) firmware stack written in ANSI C for 3-phase Permanent Magnet Synchronous Motor (PMSM) automotive traction inverters.

Designed with a modular, layered software architecture targetable to high-performance automotive microcontrollers (TI C2000, STM32G4/F4, Infineon AURIX) and structured to support ISO 26262 functional safety requirements.

---

## Architecture & System Control Loop

The firmware executes a closed-loop current control pipeline triggered by a $20\,\text{kHz}$ ePWM zero-crossing ADC Interrupt Service Routine (ISR).

```
   +---------------------------------------------------------------------------------------+
   |                                 20 kHz ADC ISR Loop                                   |
   |                                                                                       |
   |  [3-Phase Currents] -> Clarke Transform -> Park Transform -> PI Current Regulators    |
   |      (I_a, I_b, I_c)       (I_alpha, I_beta)     (I_d, I_q)         (d-axis & q-axis)    |
   |                                                                          |            |
   |  [ePWM Registers]  <-  ePWM Driver  <- SVPWM Generator <- Inv Park Transform          |
   |   (CMPA, CMPB, CMPC)   (Dead-time)     (Duty Cycles)      (V_alpha, V_beta)          |
   +---------------------------------------------------------------------------------------+

```

### Control Pipeline Steps

1. **Data Acquisition:** Multi-channel ADC reads phase currents ($i_a, i_b, i_c$) sampled at PWM zero-crossings to eliminate high-frequency switching noise.
2. **Forward Clarke Transform:** Converts 3-phase AC quantities into a 2-phase stationary orthogonal frame ($i_\alpha, i_\beta$).
3. **Forward Park Transform:** Uses rotor electrical angle ($\theta$) to map $i_\alpha, i_\beta$ into a rotating reference frame ($i_d, i_q$).
4. **PI Current Regulation:** Independent $d$-axis (flux control, setpoint $0\,\text{A}$) and $q$-axis (torque control, throttle demand) regulators generate target stator voltages ($v_d, v_q$).
5. **Inverse Park Transform:** Transforms rotating voltages ($v_d, v_q$) back to stationary reference frame ($v_\alpha, v_\beta$).
6. **Space Vector PWM (SVPWM):** Calculates duty cycles ($T_a, T_b, T_c$) with neutral-point common-mode voltage injection to maximize DC bus utilization by up to $15.5\%$.
7. **Hardware Register Load:** Converts normalized duty ratios ($0.0 \text{ to } 1.0$) into timer ticks loaded into center-aligned ePWM compare registers (`CMPA`, `CMPB`, `CMPC`).

---

## Directory Structure

```text
foc-traction-inverter-firmware/
├── motor/
│   ├── foc_math.h       # Forward/Inverse Clarke, Park, and SVPWM math routines
│   └── foc_math.c
├── drivers/
│   ├── epwm_driver.h    # Center-aligned ePWM register abstraction, dead-time, & Trip Zone
│   └── epwm_driver.c
├── control/
│   ├── pi_controller.h  # Anti-windup PI controller with voltage saturation limits
│   └── pi_controller.c
├── isr/
│   ├── adc_isr.h        # 20 kHz Hardware Interrupt Service Routine orchestration
│   └── adc_isr.c
└── main.c               # Test harness & closed-loop execution simulator

```

---

## Key Technical Features

* **Zero-Dependency Modular C Code:** Written in standard ANSI C (C99/C11) with no vendor locks, enabling easy porting to TI C2000 (CCS), STM32 (CubeIDE), or AUTOSAR Complex Device Drivers (CDD).
* **Anti-Windup PI Regulators:** Features dynamic integral clamping and voltage saturation limiting ($V_{\text{max}} = V_{\text{DC}} / \sqrt{3}$) to prevent windup during high-torque transients.
* **Hardware Dead-Time & Protection:** `epwm_driver` handles dead-time insertion to prevent shoot-through faults in high-voltage H-bridge topologies.
* **ISO 26262 Hardware Trip Zone:** Includes an immediate, asynchronous hardware trip function (`EPWM_Trip_Zone_Shutdown`) that forces all PWM outputs LOW within microseconds during overcurrent or fault conditions.

---

## Building and Running

### Prerequisites

* GCC Compiler (`gcc`)
* C standard library (`math.h`)

### Compilation

Compile all modules with optimization flags and link against the math library (`-lm`):

```bash
gcc -O2 -g main.c motor/foc_math.c drivers/epwm_driver.c control/pi_controller.c isr/adc_isr.c -o foc_sim -lm

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

--- Starting Closed-Loop Execution (Simulating 20kHz ADC ISR Ticks) ---

[ISR Tick #1]
  -> PWM Compare Registers: CMPA=2845 | CMPB=2154 | CMPC=2154

[ISR Tick #2]
  -> PWM Compare Registers: CMPA=3112 | CMPB=1887 | CMPC=1887

...

--- Testing Safety State Machine ---
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

## License

This project is open-source under the [MIT License](https://www.google.com/search?q=LICENSE&utm_source=gemini).