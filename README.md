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

<img width="1870" height="714" alt="Screenshot 2026-07-23 033347" src="https://github.com/user-attachments/assets/1b6e01e1-6cc7-427d-a03b-7e15acbacea2" />



---------------------------

<img width="1917" height="928" alt="image" src="https://github.com/user-attachments/assets/54139ecc-4b26-4ee7-859c-9a8b2a110b07" />


----------

<img width="1515" height="607" alt="image" src="https://github.com/user-attachments/assets/61e3e85d-8d31-40ee-9e19-22a177f40de6" />

---------------------------

<img width="1310" height="610" alt="Screenshot 2026-07-23 032555" src="https://github.com/user-attachments/assets/dd420071-2525-46b6-b801-e8726b61c152" />



---------------------------


<img width="1280" height="704" alt="Step_3_UR3_Excitation-ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/18539516-4ab4-4b46-a8c2-eef4200effdb" />



---------------------------

<img width="1907" height="915" alt="Screenshot 2026-07-23 032705" src="https://github.com/user-attachments/assets/68118c59-e55a-452a-8552-e88b985b4199" />


----------------------------


<img width="1907" height="912" alt="Screenshot 2026-07-23 032935" src="https://github.com/user-attachments/assets/f9b13b93-96d8-4bb9-9d16-72a74bb9167e" />


---------------------------

<img width="1328" height="666" alt="image" src="https://github.com/user-attachments/assets/9ae9f663-1323-4db1-afe9-da0c105cae3d" />




---------------------------

<img width="1552" height="697" alt="Screenshot 2026-07-23 033630" src="https://github.com/user-attachments/assets/8434464a-c31f-41d9-9113-4dd02d5cad5f" />


-----------

<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/3f375495-d4c9-4014-a7cc-05732262f748" />

----------


<img width="1915" height="917" alt="image" src="https://github.com/user-attachments/assets/2e3e14d0-ea9e-460b-a395-b047445eb40c" />



--------------

<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/b0306cd7-b487-4f8a-a17d-0c44d246d0d9" />


---------------


<img width="1917" height="927" alt="image" src="https://github.com/user-attachments/assets/6f2a72b1-8b54-45e5-8082-b6c8c3f872a6" />


--------------





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
2. The work is done in **MATLAB (R2020a ).
3. Ensure the following toolboxes are installed:
   * Simulink
   * Simscape / Simscape Multibody
   * Optimization Toolbox
   * Robotic Systems Toolbox
4. Add the root repository folder and its assets to your MATLAB path.
