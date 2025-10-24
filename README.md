# Simulink Model – Microgravity Attitude Platform

## Overview
This repository contains the **Simulink model** developed to support a two-month research internship focused on *inertia estimation for nanosatellite attitude simulators*.  
The model reproduces the dynamics of a **planar microgravity platform** designed to emulate frictionless 3D rotational motion through an **air bearing system**.  
It serves as a simulation environment to validate **inertia identification algorithms**, **Kalman filters**, and **attitude control strategies** before Hardware-in-the-Loop (HIL) implementation.

---

## Objectives
The Simulink model was created to:
- Reproduce the **rigid-body rotational dynamics** of the satellite simulator, including gravity torque, bearing friction, and external disturbances.  
- Implement a **multi-stage inertia estimation pipeline** combining:
  1. Initial nominal guess,
  2. Least Squares (LS) estimation,
  3. Instrumental Variable (IV) refinement.
- Integrate and tune an **Extended Kalman Filter (EKF)** for real-time attitude estimation using quaternion kinematics.
- Evaluate robustness against measurement noise, initial condition errors, and disturbance torques.

---

## Model Description
The top-level model (`SimplifiedModel.slx`) consists of the following main subsystems:

| Subsystem | Description |
|------------|--------------|
| **Rigid Body Dynamics** | Implements Euler’s equations of motion with gyroscopic coupling and gravity torque compensation. |
| **Reaction Wheel Actuators** | Generate control torques along the three principal axes, subject to physical saturation limits. |
| **PD Controller** | Regulates attitude error to achieve reference trajectories defined by the trajectory generator. |
| **Measurement Blocks** | Simulate noisy sensors (gyros, attitude sensors) using Gaussian noise models. |
| **Inertia Estimation Module** | Executes LS and IV algorithms to identify the six independent parameters of the inertia matrix. |
| **Extended Kalman Filter (EKF)** | Performs real-time attitude estimation using quaternion-based dynamics. |

---

## Inertia Estimation Pipeline
The estimation relies on the reformulation of Euler’s equation:
\[
\tau = J \dot{\omega} + \omega \times (J \omega)
\]
which is linear in the parameters of \(J\).  
The pipeline proceeds as:
1. **Least Squares (LS)** estimation from noisy measurements.  
2. **Instrumental Variable (IV)** refinement to ensure unbiased results in presence of correlated noise.  
3. **Closed-loop auxiliary model** to generate consistent instruments \(Z(t)\) via a noise-free response simulation.

This process allows sub-percent accuracy in estimated inertia values even with model uncertainties.

---

## Extended Kalman Filter (EKF)
The EKF is formulated over the 7-state vector:
\[
x = [\omega_x, \omega_y, \omega_z, q_0, q_1, q_2, q_3]^T
\]
and performs:
- **Prediction:** Numerical integration of rotational dynamics.  
- **Update:** Measurement correction using noisy angular velocity and attitude data.  

The filter is tuned via a proportional rule:
\[
Q = \lambda R
\]
where \( \lambda \) controls the trade-off between model confidence and measurement trust.  
Proper tuning ensures convergence without divergence or bias accumulation.

---

## Requirements
- MATLAB / Simulink **R2023b or later**
- Toolboxes:
  - Simscape Multibody
  - Aerospace Blockset
  - Control System Toolbox
  - Signal Processing Toolbox

---

## How to Run
1. Open MATLAB and navigate to the project folder.
2. Run the initialization script:
   ```matlab
   SimplifiedModelData
   ```
3. Open the main Simulink model:
   ```matlab
   open_system('SimplifiedModel.slx')
   ```
4. Launch simulation and visualize the results:
   ```matlab
   sim('SimplifiedModel')
   ```

---

## Author
**Massimo De Bellis Vitti**  
M.Sc. Space Engineering – Politecnico di Milano  
Double Degree at ISAE-SUPAERO, Toulouse, France  
📧 massimo.de-bellis-vitti@student.isae-supaero.fr

---

## Reference
This model was developed as part of the research project:  
> *“Estimation of the Inertia of an Experimental Satellite Simulator”*  
> Département Conception et Conduite des Véhicules Aéronautiques et Spatiaux (DCAS), ISAE-SUPAERO, 2025.

---

## License
This repository is released for academic and research use only.  
Please cite the above report when using or modifying the model.
