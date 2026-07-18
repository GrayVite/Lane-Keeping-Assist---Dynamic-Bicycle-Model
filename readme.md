# Lane Keeping Assist (LKA) System Simulation

**Modeling & Simulation of an LQR-Controlled LKA System Using Dynamic and Kinematic Bicycle Models**

## Overview

This repository contains the Module 3 deliverables for an academic Lane Keeping Assist (LKA) system project. Modules 1 and 2 established the system configuration and mathematical models. Module 3 focuses on translating those models into a closed-loop simulation environment in **MATLAB/Simulink**.

The simulation visualizes and analyzes continuous lane-keeping performance of a 2026 Toyota Corolla Altis X 1.6 Manual under realistic conditions, including sensor noise and external disturbances.

### Key Features
- Dynamic Bicycle Model (4-state) with full lateral dynamics
- Kinematic Bicycle Model (2-state) for geometric baseline comparison
- Linear Quadratic Regulator (LQR) optimal control
- Sensor noise injection (30 FPS monocular camera simulation)
- Three driving scenarios: baseline recovery, disturbance rejection, and curvature tracking
- Analysis of failure modes and proposed optimizations

---

## System Architecture

### Vehicle Model
Both models represent a **2026 Corolla Altis X 1.6 Manual**.

| Aspect      | Dynamic Bicycle Model                        | Kinematic Bicycle Model               |
| ----------- | -------------------------------------------- | ------------------------------------- |
| States      | $[\dot{y},\ \dot{\psi},\ e_y,\ e_\psi]^T$ | $[e_y,\ e_\psi]^T$               |
| Assumptions | Includes mass, yaw inertia, tire slip forces | Zero tire slip, pure geometric motion |
| Realism     | High (suitable for production analysis)      | Idealized baseline                    |

### Control Design
- **Controller**: Linear Quadratic Regulator (LQR)
- **Cost Function**:  
  $`
J = \int_0^\infty (x^T Q x + u^T R u)\, dt
  `$
- **Weighting Matrices**:
  - **Dynamic**: $Q = \mathrm{diag}(1,\ 1,\ 100,\ 10)$, $R = 1$
  - **Kinematic**:  
  $`Q = \begin{bmatrix} 100 & 0 \\ 0 & 10 \end{bmatrix},\quad R = 1`$
  
- Optimal gain (Dynamic model):  
  $K = [0.7438,\ 0.8688,\ 10.0002,\ 3.182]$

### Simulation Environment
- Platform: MATLAB + Simulink
- Longitudinal velocity: Constant \( $v_x = 20$ \) m/s (highway speed)
- Sensor noise: Band-limited white noise, sample time 0.033 s (30 FPS)
- Disturbances: Lateral force (crosswind) and road curvature

---

## Simulation Scenarios

### Scenario A – Baseline Recovery
- Initial lateral offset: \( 0.3 \) m  
- Initial heading error: \( 0.05 \) rad (~2.9°)  
- **Result**: Lateral error settles to 0 m within ~1.0 s (Dynamic) / <0.2 s (Kinematic)
  <img width="1679" height="1011" alt="Scenario1_baseline" src="https://github.com/user-attachments/assets/80cb3e6c-daef-422c-8e00-e22f4a1f3928" />


### Scenario B – Disturbance Rejection
- 5000 N lateral force (severe crosswind) applied at \( t = 5 \) s  
- **Result**: Peak lateral overshoot limited to ~0.03 m; robust recovery
  <img width="1679" height="1011" alt="Scenario2_LateralForce" src="https://github.com/user-attachments/assets/e7eebd1a-ada9-4339-af84-3e350ab83d0c" />


### Scenario C – Curvature Tracking
- Constant radius curve: \( R = 500 \) m at 20 m/s  
- **Result**: Steady-state lateral error ≈ 0.005 m (Dynamic) / −0.00054 m (Kinematic)  
  (Expected due to lack of integral action in standard LQR)
  <img width="1679" height="1011" alt="Scenario3_Curvature" src="https://github.com/user-attachments/assets/5de4e32b-f1c0-4429-8adf-a406f1bc8d0c" />


## Getting Started

### Prerequisites
- MATLAB R2021a or later (with Simulink)
- Control System Toolbox (for LQR and state-space design)

### Running the Simulations
1. Clone the repository:
   ```bash
   git clone https://github.com/GrayVite/Lane-Keeping-Assist---Dynamic-Bicycle-Model.git
   ```
2. Open MATLAB and navigate to the project root.
3. Run the initialization script for the desired model:
   ```matlab
   cd Dynamic_Model
   init_params          % Loads A, B, Q, R, K, vehicle parameters
   open_system('LKA_Dynamic')
   ```
4. Run the Simulink model (▶ button) and examine the scopes / logged timeseries.

---

## Key Results Summary

| Metric                        | Dynamic Model      | Kinematic Model     |
|-------------------------------|--------------------|---------------------|
| Baseline settling time        | ~1.0 s             | < 0.2 s             |
| Peak disturbance overshoot    | ~0.03 m            | N/A                 |
| Curvature steady-state error  | ~0.005 m           | −0.00054 m          |
| Realism                       | High               | Idealized           |

**Conclusion**: The kinematic model overestimates performance by neglecting inertia and tire forces. The dynamic bicycle model is essential for realistic evaluation of actuator limits, high-speed stability, and sensor degradation.

---

## Limitations & Future Work

### Current Limitations
- No actuator slew-rate constraints
- Fixed-speed LQR gains (not gain-scheduled)
- Direct noisy camera feedback (no state estimation)
- Standard LQR lacks integral action → residual curve-tracking error

### Proposed Optimizations
1. **Linear Quadratic Integral (LQI)** – Eliminate steady-state error on curves
2. **Gain Scheduling** – Pre-compute \( K \) matrices across 5–40 m/s
3. **Kalman Filter** – Robust state estimation and sensor-dropout handling
