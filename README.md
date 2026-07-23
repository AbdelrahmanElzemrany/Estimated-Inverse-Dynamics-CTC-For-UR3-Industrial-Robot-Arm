# Estimated Inverse Dynamics Model-Based Computed Torque Controller (CTC) for UR3 Robot Arm

An end-to-end parameter estimation and feedback linearization framework for a 6-DOF UR3 industrial robotic manipulator. This project implements a high-precision Computed Torque Controller (CTC) using an experimentally identified plant model that includes non-linear link dynamics and friction effects.

---

## 📝 Project Overview

Model-free control strategies (like independent joint PD loops) experience high tracking errors when handling dynamic loads and gravitational forces. Model-based control architectures resolve this by calculating the required dynamic torques on the fly to cancel out the robot's physical properties. 

This repository provides a step-by-step pipeline to handle the entire control loop lifecycle: from trajectory excitation and physical parameter estimation to closed-loop validation and computed torque implementation.

### Key Features
* **Trajectory Optimization:** Excitation trajectories designed to maximize parameter visibility.
* **Physical Consistency:** Parameter estimation constraints that eliminate physically impossible variables (e.g., negative mass or inertia).
* **Friction Identification:** Integrates Coulomb and viscous friction models into the joint dynamics.
* **Feedback Linearization:** Decouples and linearizes the highly coupled multi-variable 6-DOF system.

---
---------------------------

<img width="1310" height="610" alt="Screenshot 2026-07-23 032555" src="https://github.com/user-attachments/assets/dd420071-2525-46b6-b801-e8726b61c152" />



---------------------------


<img width="1280" height="704" alt="Step_3_UR3_Excitation-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/5c831a5e-3d07-4cc3-821f-abdc7c86dcf0" />



---------------------------

<img width="1907" height="915" alt="Screenshot 2026-07-23 032705" src="https://github.com/user-attachments/assets/68118c59-e55a-452a-8552-e88b985b4199" />


----------------------------


<img width="1907" height="912" alt="Screenshot 2026-07-23 032935" src="https://github.com/user-attachments/assets/f9b13b93-96d8-4bb9-9d16-72a74bb9167e" />


---------------------------
## 🛠 File Architecture & Workflow Pipeline

To run this project successfully, execute the scripts and Simulink files in the following sequential order:

### Phase 1: Symbolic Modeling & Trajectory Generation
* **`Step_1_TheRegressorModel.m`**: Derives and loads the symbolic regressor matrix and structural properties of the UR3 arm.
* **`Step_2_Optimized_Trajectory.m`**: Generates and optimizes the joint exciting profiles needed to expose the inertial variables.

### Phase 2: System Identification Data Loop
* **`Step_3_UR3_Excitation.slx`**: Simulates the robot running the optimized excitation path to log physical feedback.
* **`Step_4_UR3_Data_Extraction.slx`**: Preprocesses and extracts the Position, Velocity, and Acceleration (PVA) states along with joint torques.
* **`Step_5_Constrained_Estimation.m`**: Runs the optimization routine to find the physically consistent base parameters.

### Phase 3: Validation & Formulation
* **`Step_6_UR3_Validation.slx`**: Evaluates the estimated plant model against separate validation trajectories.
* **`Step_7_UR3_Vaildation_Extraction.slx`**: Extracts validation error logs from the testing workspace.
* **`Step_8_Validation_results.m`**: Computes and plots tracking residuals to confirm model accuracy.
* **`Step_9_Estimated_INV_Matrices_reform.m`**: Formulates the final validated inverse dynamics matrices for real-time control.

### Phase 4: Model-Based Control Testing
* **`Step_10_UR3_CTC_IMF.slx`**: Implements Computed Torque Control featuring the Inverse Model Feedforward loop.
* **`Step_10_UR3_CTC_SF.slx`**: Alternative Computed Torque Control structural implementation framework.

---

## 📐 Mathematical Framework

The controller calculates real-time actuator torques ($\tau$) by balancing the identified inertial, centrifugal, gravitational, and frictional properties:

$$\tau = M(q)\left[\ddot{q}_d + K_p e + K_d \dot{e}\right] + C(q, \dot{q})\dot{q} + G(q) + F(\dot{q})$$

<img width="1558" height="724" alt="Screenshot 2026-07-23 030937" src="https://github.com/user-attachments/assets/17d2c9c7-2369-4d90-be4d-6a26b23b0a1a" />


Where:
* $M(q)$ is the identified bounded Mass/Inertia matrix.
* $C(q, \dot{q})$ captures Coriolis and centrifugal effects.
* $G(q)$ handles real-time gravity cancellation.
* $F(\dot{q})$ compensates for Coulomb and viscous friction.

---

## 💻 Requirements & Setup

1. Clone this repository to your local directory.
2. Open **MATLAB (R2021a or newer recommended)**.
3. Ensure the following toolboxes are installed:
   * Simulink
   * Simscape / Simscape Multibody
   * Optimization Toolbox
   * Robotic Systems Toolbox
4. Add the root repository folder and its assets to your MATLAB path.
