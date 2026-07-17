# Lane Keeping Assist (LKA) System Simulation

**Modeling & Simulation of an LQR-Controlled LKA System Using Dynamic and Kinematic Bicycle Models**

College of Electrical and Mechanical Engineering (CEME), NUST  
Mechatronics Department | Degree-45 | Syndicate-A  
Course: Modeling and Simulation (M&S-321)

**Team Members**
- Muhammad Saqlain (482581)
- Muhammad Awais (482577)
- Basam Murtaza (463884)
- Ayan Ahmad Khan (455934)

---

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
| States      | \( [ \dot{y}, \dot{\psi}, e_y, e_\psi ]^T \) | \( [ e_y, e_\psi ]^T \)               |
| Assumptions | Includes mass, yaw inertia, tire slip forces | Zero tire slip, pure geometric motion |
| Realism     | High (suitable for production analysis)      | Idealized baseline                    |

### Control Design
- **Controller**: Linear Quadratic Regulator (LQR)
- **Cost Function**:  
  \[
  J = \int_0^\infty (x^T Q x + u^T R u)\, dt
  \]
- **Weighting Matrices**:
  - Dynamic: \( Q = \operatorname{diag}(1, 1, 100, 10) \), \( R = 1 \)
  - Kinematic: \( Q = \begin{bmatrix} 100 & 0 \\ 0 & 10 \end{bmatrix} \), \( R = 1 \)
- Optimal gain (Dynamic model):  
  \( K = [0.7438,\ 0.8688,\ 10.0002,\ 3.182] \)

### Simulation Environment
- Platform: MATLAB + Simulink
- Longitudinal velocity: Constant \( v_x = 20 \) m/s (highway speed)
- Sensor noise: Band-limited white noise, sample time 0.033 s (30 FPS)
- Disturbances: Lateral force (crosswind) and road curvature

---

## Simulation Scenarios

### Scenario A – Baseline Recovery
- Initial lateral offset: \( 0.3 \) m  
- Initial heading error: \( 0.05 \) rad (~2.9°)  
- **Result**: Lateral error settles to 0 m within ~1.0 s (Dynamic) / <0.2 s (Kinematic)

### Scenario B – Disturbance Rejection
- 5000 N lateral force (severe crosswind) applied at \( t = 5 \) s  
- **Result**: Peak lateral overshoot limited to ~0.03 m; robust recovery

### Scenario C – Curvature Tracking
- Constant radius curve: \( R = 500 \) m at 20 m/s  
- **Result**: Steady-state lateral error ≈ 0.005 m (Dynamic) / −0.00054 m (Kinematic)  
  (Expected due to lack of integral action in standard LQR)

---

## Project Structure
