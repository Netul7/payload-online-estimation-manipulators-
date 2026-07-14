# 🤖 Online Payload Parameter Estimation for Robotic Manipulators

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-orange?style=flat-square&logo=mathworks)
![Simulink](https://img.shields.io/badge/Simulink-Simulation-blue?style=flat-square&logo=mathworks)
![C++](https://img.shields.io/badge/C++-17-00599C?style=flat-square&logo=c%2B%2B)
![Hardware](https://img.shields.io/badge/Hardware-Franka%20Research%203-red?style=flat-square)

## 📌 Executive Summary
In robotic manipulation, unknown payloads significantly degrade trajectory tracking and control stability. This repository presents the implementation and experimental validation of an advanced online parameter identification framework based on the **Modified Dynamic Regressor Extension and Mixing (MDREM)** algorithm—a novel enhancement of the DREM technique designed to guarantee fast, and decoupled parameter convergence.

The MDREM framework is systematically validated across two distinct robotic architectures with different estimation objectives:

1. **Geomagic Touch (simplified 3-DOF Haptic Device) — Full & Reduced System Identification:**
   * **Objective:** Simultaneous online estimation of internal robot dynamics and payload properties under a reduced-parameter geometric formulation.
   * **Workflow:** Full parameter convergence is demonstrated within a **MATLAB/Simulink simulation environment**. In physical laboratory trials, where geometric simplifications introduce real-world modeling uncertainties, the algorithm successfully validates the estimation of **gravitational dynamic parameters**, demonstrating robustness against unmodeled dynamics.

2. **Franka Research 3 (FR3, 7-DOF Cobot) — Robust Online Payload Estimation:**
   * **Objective:** Real-time identification of unknown **payload inertial parameters** (mass, center of mass, and inertia tensor) mounted on the end-effector.
   * **Workflow:** Direct hardware deployment utilizing baseline robot dynamic parameters derived from Franka Emika Panda literature. Despite the inherent parametric mismatch between robot versions, the MDREM algorithm demonstrates high robustness, achieving fast and stable convergence to a close neighborhood of the true payload parameters without instability.
---

## 🧮 Methodology & Theoretical Framework: The MDREM Algorithm

Standard parameter estimation methods (like RLS or gradient-descent) rely heavily on strict **Persistency of Excitation (PE)** conditions over the entire trajectory and suffer from parameter coupling, meaning poor excitation in one degree of freedom degrades the convergence of all parameters.

This project implements a **Modified Dynamic Regressor Extension and Mixing (MDREM)** approach, an advanced enhancement of the DREM technique, to overcome these limitations and achieve decoupled, finite-time parameter convergence.

### 1. Dynamic Regressor Parametrization & Real-World Uncertainty
The manipulator's rigid-body dynamics are linearly parametrized in terms of an unknown constant parameter vector `pi`:

`tau - C(q, q_dot)*q_dot - g(q) = Y(q, q_dot, q_ddot) * pi`

A core strength of this experimental validation lies in evaluating algorithm resilience under real-world modeling discrepancies:
* **Geomagic Touch:** A reduced-order geometric formulation introduces structural uncertainty. While simulations prove exact theoretical convergence, laboratory trials validate the robust identification of dominant gravitational terms.
* **Franka Research 3 (FR3):** Baseline robot dynamics are compensated using literature-reported parameters from the predecessor model (Franka Emika Panda). Rather than requiring a recalibrated baseline, the MDREM estimator proves robust against this baseline mismatch, successfully driving the payload parameter estimation errors to a bounded, practically negligible neighborhood of the true values.

### 2. Extension and Mixing (The DREM Principle)
To decouple the parameter vector, the initial regressor equation is extended by applying stable Linear Time-Invariant (LTI) filtering operators, generating an extended square system:

`Y_ext(t) * pi = tau_ext(t)`

By premultiplying the extended system by the adjugate matrix `adj(Y_ext(t))`, the system is algebraically transformed into `q` **independent scalar equations**:

`phi_i(t) * pi_i = y_i(t),   for i = 1, 2, ..., q`

Where `phi_i(t) = det(Y_ext(t))` is the scalar regressor. 

### 3. The MDREM Enhancement
While standard DREM decouples the system, convergence speed is still tied to the determinant's behavior. The **Modified DREM (MDREM)** introduced in this work implements an optimized adaptation law and filtering architecture that:
1. **Accelerates convergence rates** for individual physical parameters without increasing high-frequency noise amplification.
2. **Relaxes excitation requirements**, allowing the identification of critical payload parameters (such as mass `m` and center of mass `r_com`) even under smooth, non-aggressive industrial trajectories.

---

## 🔬 Experimental Platforms & Validation

| Platform | Role in Project | Environment / Stack | Experimental Scope & Status |
| :--- | :--- | :--- | :--- |
| **Geomagic Touch** | Simulation vs. Hardware Reality | MATLAB / Simulink | ✔️ **Sim:** Full Parameter ID<br>✔️ **Hardware:** Gravitational Parameters Validated |
| **Franka Research 3** | Unconstrained Payload ID | C++ / [Middleware: ROS2/libfranka] | ✔️ **Hardware:** Full Payload ID (No simplifications) |

> **Note on Geomagic Touch implementation:** Due to laboratory intellectual property guidelines, the specific low-level control code for the Geomagic Touch is not publicly shared. This repository includes the complete, author-developed MATLAB/Simulink simulation environment, along with the physical experimental data curves proving algorithm convergence.

---

## 📊 Key Results & Convergence

*(Instructions: Replace the text below with actual images or animated GIFs showing your results)*

### 1. Simulink Model vs. Experimental Data (Geomagic Touch)
![Geomagic Comparison](path/to/your/image_or_plot.png)
*Figure 1: Comparison between simulation trajectory tracking and real hardware execution.*

### 2. Robust Payload Parameter Convergence (Franka Research 3)
![Franka Convergence Plot](path/to/your/image_or_gif.gif)
*Figure 2: Online estimation of payload inertia parameters under baseline robot model uncertainty. The estimated values rapidly converge to a close, bounded neighborhood of the true payload metrics within [X] seconds of dynamic trajectory execution, demonstrating strong robustness against uncompensated baseline dynamics.*

---

## 🗂️ Repository Structure

```text
├── docs/                        # Mathematical derivations, block diagrams, and thesis summary
├── 1_geomagic_touch_sim/        # MATLAB/Simulink models and experimental result validation
│   ├── simulink_models/         # .slx files for manipulator and estimator
│   ├── matlab_scripts/          # Initialization, regressor computation, and plotting (.m)
│   └── experimental_results/    # Lab data comparisons (CSV/Mat)
└── 2_franka_research_hardware/  # C++ implementation for Franka Research 3
    ├── src/                     # Core estimation algorithm and control loops
    ├── include/                 # Header files and matrix definitions
    └── config/                  # Estimator gains and trajectory parameters
