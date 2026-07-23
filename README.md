# Estimated Inverse Dynamics Model-Based Computed Torque Controller (CTC) for UR3 Robot Arm

An end-to-end parameter estimation and feedback linearization framework for a 6-DOF UR3 industrial robotic manipulator. This project implements a high-precision Computed Torque Controller (CTC) using an experimentally identified plant model that includes non-linear link dynamics and friction effects.

---

## 📝 Project Overview

Model-free control strategies (like independent joint PD loops) experience high tracking errors when handling dynamic loads and gravitational forces. Model-based control architectures resolve this by calculating the required dynamic torques on the fly to cancel out the robot's physical properties. 





This repository provides a step-by-step pipeline to handle the entire control loop lifecycle: from trajectory excitation and physical parameter estimation to closed-loop validation and computed torque implementation.

## 🧠 System Identification & Grey-Box Modeling

The dynamic parameters of the UR3 industrial robot are identified using an experimental grey-box parameter estimation approach. 

### The Estimation Philosophy
1. **Geometric Priors Only:** It is assumed that only the kinematic structure—defined by the **Modified Denavit-Hartenberg (MDH)** convention—is known.
2. **Hidden Plant Parameters:** The true physical properties (mass $m_i$, center of mass $P_{c_i}$, inertia tensors $I_i$, and joint friction coefficients $F_v, F_c$) are treated as completely unknown.
3. **Simscape Data Collection:** The high-fidelity Simscape Multibody model serves as the physical experimental plant. By executing optimized, highly exciting trajectories on the Simscape model, we log joint position, velocity, acceleration (PVA), and torque profiles ($\tau$).
4. **Constrained Least Squares Optimization:** The collected data is fed into a symbolic regressor model matrix $Y(q, \dot{q}, \ddot{q})$ to reconstruct and estimate a physically consistent set of parameters, ensuring that the estimated inertia tensors remain positive-definite and mass values stay positive.

> [!TIP]
> ### 💡 Note on Base Parameter Identification
> In robotic parameter estimation, not all inertial parameters can be isolated independently due to linear dependencies in the regressor matrix (caused by the physical trajectory limitations and joint constraints). 
> 
> To resolve this, this project utilizes **Base Parameter Identification**. We compute the **minimal set of parameters** (linear combinations of the original link inertias, masses, and offsets) to eliminate unobservable parameters and guarantee a full-rank regressor matrix. Only this identifiable minimal set is estimated to ensure numerical stability and flawless tracking performance in the Computed Torque Controller (CTC).

### ✅ Model Validation Technique
To guarantee that the identified base parameters are correct, robust, and free from overfitting, the estimated model is validated using a cross-validation technique:
* **Distinct Trajectories:** The plant is subjected to **two entirely different validation trajectories** that were never seen during the parameter excitation phase.
* **Torque Tracking Residuals:** The actual joint torques logged from the Simscape plant are directly compared against the predicted torques computed by the estimated parameter matrix.
* **Accuracy Confirmation:** A low residual error across both independent test sets mathematically verifies that the identified minimal parameter set accurately captures the true structural dynamics of the physical arm under any motion profile.

### 🛠️ Controller Reconstruction & Deployment
Once the identified base parameters successfully pass the cross-validation tests, they are finalized and injected into the dynamic framework:
* **Matrix Reformulation:** The validated minimal parameter set is used to reconstruct the high-fidelity estimated inverse dynamics matrices: the Mass/Inertia matrix $\hat{M}(q)$, the Coriolis/Centrifugal matrix $\hat{C}(q, \dot{q})$, and the Gravity vector $\hat{G}(q)$.
* **Model-Based CTC Design:** These reconstructed matrices act as the core mathematical engine for the model-based **Computed Torque Controller (CTC)**. By executing feedback linearization in real-time, the controller cancels out the non-linear multibody dynamics, leaving decoupled linear joint loops that achieve exceptional path-tracking accuracy.

* ## 📐 Mathematical Framework

The controller calculates real-time actuator torques ($\tau$) by balancing the identified inertial, centrifugal, gravitational, and frictional properties:

$$\tau = M(q)\left[\ddot{q}_d + K_p e + K_d \dot{e}\right] + C(q, \dot{q})\dot{q} + G(q) + F(\dot{q})$$

<img width="1558" height="724" alt="Screenshot 2026-07-23 030937" src="https://github.com/user-attachments/assets/17d2c9c7-2369-4d90-be4d-6a26b23b0a1a" />


Where:
* $M(q)$ is the identified bounded Mass/Inertia matrix.
* $C(q, \dot{q})$ captures Coriolis and centrifugal effects.
* $G(q)$ handles real-time gravity cancellation.
* $F(\dot{q})$ compensates for Coulomb and viscous friction.

---

> [!NOTE]
> ### 🔩 Friction Integration into the Coriolis Matrix
> For structural and implementation elegance within the real-time control loop, the estimated joint friction models ($F_v$ and $F_c$) are mathematically grouped and embedded directly into the **Coriolis/Centrifugal matrix** $\hat{C}(q, \dot{q})$. This consolidates all velocity-dependent dynamic losses into a unified matrix expression, optimizing real-time calculation efficiency in the controller.



### Denavit-Hartenberg (DH) Parameters (Modified)

           

| Link ($i$) | $\alpha_{i-1}$ (Link Twist) | $a_{i-1}$ (Link Length) | $d_i$ (Link Offset) | $\theta_i$ (Joint Angle) |
| :---: | :---: | :---: | :---: | :---: |
| **1** | $0$ | $0$ | $0.15190$ | $q_1$ |
| **2** | $\pi/2$ | $0$ | $0$ | $q_2$ |
| **3** | $0$ | $0.24365$ | $0$ | $q_3$ |
| **4** | $0$ | $0.21325$ | $0.11235$ | $q_4$ |
| **5** | $\pi/2$ | $0$ | $0.08535$ | $q_5$ |
| **6** | $-\pi/2$ | $0$ | $0.0819$ | $q_6$ |

-----------------------------------



### Joint Friction Parameters

| Joint ($i$) | Viscous Friction Coefficient ($F_v$) | Coulomb Friction Coefficient ($F_c$) |
| :---: | :---: | :---: |
| **1** | `1.2` | `0.8` |
| **2** | `1.2` | `0.8` |
| **3** | `1.2` | `0.8` |
| **4** | `0.2` | `0.2` |
| **5** | `0.2` | `0.15` |
| **6** | `0.1` | `0.1` |

---------------------------

## 🖼️ System Visualizations & Simulation Plots-

<img width="1870" height="714" alt="Screenshot 2026-07-23 033347" src="https://github.com/user-attachments/assets/1b6e01e1-6cc7-427d-a03b-7e15acbacea2" />

Figure 1 : The UR3 simscape model .

---------------------------



<img width="1515" height="607" alt="image" src="https://github.com/user-attachments/assets/61e3e85d-8d31-40ee-9e19-22a177f40de6" />


Figure 2 : The friction model add at each joint to provide additional nonlinearity .

---------------------------


<img width="1917" height="928" alt="image" src="https://github.com/user-attachments/assets/54139ecc-4b26-4ee7-859c-9a8b2a110b07" />


Figure 3 : The optimized excitation trajectory to ensure all the base parameters to be estimated excited also to get a low condition number of the regressor matrix to get the correct results .

-------------------------

<img width="1310" height="610" alt="Screenshot 2026-07-23 032555" src="https://github.com/user-attachments/assets/dd420071-2525-46b6-b801-e8726b61c152" />

Figrue 4 : The excitation experiment configuration .

---------------------------


<img width="1280" height="704" alt="Step_3_UR3_Excitation-ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/18539516-4ab4-4b46-a8c2-eef4200effdb" />

Figure 5 : The UR3 robot arm is moving according to the excitation trajectory .

---------------------------
<img width="1917" height="928" alt="image" src="https://github.com/user-attachments/assets/92cce9d8-f4cb-4de7-ac49-476b09f116ad" />


Figure 6 : The first cross validation plot against slow validation trajectory .

----------------------------

**Figure 6 Caption:** Cross-validation results showing per-joint fit percentages 
and overall system accuracy (99.97%).


### Quantitative Cross-Validation Results

| Joint | Fit Percentage |
| :--- | :---: |
| Joint 1 (Base Yaw) | 99.89% |
| Joint 2 (Shoulder) | 99.94% |
| Joint 3 (Elbow) | 99.95% |
| Joint 4 (Wrist 1) | 99.83% |
| Joint 5 (Wrist 2) | 99.31% |
| Joint 6 (Wrist 3) | 99.52% |
| **Overall Accuracy** | **99.97%** |

> **Note:** The overall system cross-validation accuracy outperforms individual joint predictions due to error-canceling geometric alignments within the multi-body system structure during dynamic state estimation.



<img width="1907" height="912" alt="Screenshot 2026-07-23 032935" src="https://github.com/user-attachments/assets/f9b13b93-96d8-4bb9-9d16-72a74bb9167e" />

Figure 7 The second cross validation against fast and aggresssive trajectory

---------------



**Figure 7 Caption:** Cross-validation results showing per-joint fit percentages 
and overall system accuracy (99.89%).

### Quantitative Cross-Validation Results

| Joint | Fit Percentage |
|-------|----------------|
| Joint 1 (Base Yaw) | 99.66% |
| Joint 2 (Shoulder) | 99.91% |
| Joint 3 (Elbow) | 99.83% |
| Joint 4 (Wrist 1) | 99.50% |
| Joint 5 (Wrist 2) | 98.91% |
| Joint 6 (Wrist 3) | 98.55% |
| **Overall Accuracy** | **99.89%** |



---------------------------


<img width="1328" height="666" alt="image" src="https://github.com/user-attachments/assets/9ae9f663-1323-4db1-afe9-da0c105cae3d" />


Figure 8 : The Model-Based computed torque controller (CTC) surface level .

---------------------------

<img width="1552" height="697" alt="Screenshot 2026-07-23 033630" src="https://github.com/user-attachments/assets/8434464a-c31f-41d9-9113-4dd02d5cad5f" />

Figure 9 : THe computed torque controller configuration with S-functions estimated inverse dynamics matrices . 

-----------

<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/3f375495-d4c9-4014-a7cc-05732262f748" />

Figure 10 : Testing the overall system against different trajectories .


----------

<img width="1915" height="917" alt="image" src="https://github.com/user-attachments/assets/2e3e14d0-ea9e-460b-a395-b047445eb40c" />


Figure 11 : The desired joint position commands and the tracking error of the joints plot .

--------------

<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter (1)" src="https://github.com/user-attachments/assets/b0306cd7-b487-4f8a-a17d-0c44d246d0d9" />

Figure 12 : Testing the robustness of the system and the stablity of it joints .

---------------


<img width="1917" height="927" alt="image" src="https://github.com/user-attachments/assets/6f2a72b1-8b54-45e5-8082-b6c8c3f872a6" />

Figure 13 : The tracking error across all the joints remains low 

--------------



<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter (2)" src="https://github.com/user-attachments/assets/757413ea-0d10-48bb-86b3-908fdea7926c" />
Figure 14 : Testing the performance of the wrist joints



----------------------------------------------

<img width="1535" height="900" alt="image" src="https://github.com/user-attachments/assets/ba6e63d9-6d92-492c-94cf-796b4e678b7e" />

Figure 15 : The tracking error across the joints remains low but it also show why the sixith joint got the least validation percentage 

------------------------

<img width="1280" height="704" alt="Step_10_UR3_CTC_SF-ezgif com-video-to-gif-converter (3)" src="https://github.com/user-attachments/assets/182aa971-a6bc-452b-a935-2ad12b29aed8" />


Figure 16 : Testing the all joints with sinesudal agressive trejactory 



----------------------


<img width="1532" height="868" alt="image" src="https://github.com/user-attachments/assets/8572a7b8-4c2e-46b5-b527-73c6980f5eb4" />

Figure 17 : The tracking error across the joints remains very low and converges to zero but the wrist joints get a decrease in performance






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



## 💻 Requirements & Setup

1. Clone this repository to your local directory.
2. The work is done in **MATLAB (R2020a ).
3. Ensure the following toolboxes are installed:
   * Simulink
   * Simscape / Simscape Multibody
   * Optimization Toolbox
   * Robotic Systems Toolbox
4. Add the root repository folder and its assets to your MATLAB path.
