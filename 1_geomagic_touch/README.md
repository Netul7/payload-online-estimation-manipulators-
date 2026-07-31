
# 🕹️ Geomagic Touch: Simulation & Experimental Validation

This module contains the high-fidelity **MATLAB/Simulink simulation environment** and the **experimental data validation** for the full system parameter identification (robot + payload) using the MDREM algorithm on the Geomagic Touch haptic device.

> ⚠️ **Implementation Note:** Due to laboratory intellectual property guidelines, the low-level C++ control firmware for the physical Geomagic Touch is not publicly shared. However, the complete simulation environment (which mimics the real system dynamics) and the experimental data logs proving hardware convergence are fully provided here.

---

## ⚙️ Technical Implementation Details

To keep this documentation highly scannable, detailed kinematic and dynamic modeling assumptions are collapsed below. Click to expand:

<details>
<summary><b>1. Kinematics & Denavit-Hartenberg (DH) Parameters</b></summary>
<br>

Recall that the Geomagic Touch was modeled using a simplified three-degree-of-freedom (3-DoF) configuration. The figure below details this standard Denavit–Hartenberg (DH) geometric setup, and the subsequent table presents the corresponding DH parameters.

<div align="center">
  <img src="../Images/D-H configuration.jpg" alt="Description" width="500">
  <p><em>Figure 1.1. Denavit-Hartenberg configuration of the simplified 3 DoF Geomagic touch.</em></p>
</div>

| Joint | $a_i$ [m] | $d_i$ [m] | $\alpha_i$ [rad] | $q_i$ [rad] |
|-------|-----------|-----------|------------------|-------------|
| 1     | 0         | 0         | $\pi/2$          | $q_1^*$     |
| 2     | 0.145     | 0         | 0                | $q_2^*$     |
| 3     | 0.1738    | 0         | 0                | $q_3^*$     |

$*$ variable quantity

</details>

<details>
<summary><b>2. Dynamic Model & Geometric Simplifications</b></summary>
<br>
  
To achieve a reduced-order parameter formulation, several geometric simplifications were applied to the manipulator's links and payload. To see the full model derivation and  assumptions consult the master's thesis here [view thesis](../docs/Master_thesis_Ernesto.pdf)

The dynamic model then is given by

$$\boldsymbol{H}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q},\dot{\boldsymbol{q}})\dot{\boldsymbol{q}}+\boldsymbol{D} \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}$$

The inertia matrix $\boldsymbol{H}(\boldsymbol{q})$ is now given by

<div align="center">
  <img src="../Images/equations/Inertia_matrix.png" alt="Description" width="500">
  <p><em></em></p>
</div>

<div align="center">
  <img src="../Images/equations/Coriolis_matrix.png" alt="Description" width="500">
  <p><em></em></p>
</div>

<div align="center">
  <img src="../Images/equations/coeff_fric_mat.png" alt="Description" width="500">
  <p><em></em></p>
</div>

<div align="center">
  <img src="../Images/equations/gravity_vetor.png" alt="Description" width="500">
  <p><em></em></p>
</div>

<div align="center">
  <img src="../Images/equations/parameters_vector.png" alt="Description" width="500">
  <p><em></em></p>
</div>


* **Simulation:** The MDREM algorithm successfully identifies the full parameter vector $`\theta`$ within the ideal Simulink environment.
* **Hardware Validation:** In physical deployment, the geometric simplifications introduce structural modeling uncertainty. Consequently, the experimental validation successfully isolates and identifies the **gravitational dynamic parameters**, which dominate the low-speed static behavior of the manipulator.
</details>

<details>
<summary><b>3. Linear parameterization</b></summary>
<br>
  
</details>

---

## 📂 Module Structure

* `/matlab_scripts/`: Contains initialization scripts (`init_params.m`), MDREM filter definitions, and plotting utilities.
* `/simulink_models/`: Contains the `.slx` block diagrams for the rigid body plant, trajectory generator, and the MDREM estimator.
* `/experimental_results/`: Contains `.csv`/`.mat` datasets collected from the physical lab trials and scripts to plot real-world parameter convergence.

---

## 🚀 Getting Started (Running the Simulation)

To test the estimation algorithm in simulation:

1. **Prerequisites:** Ensure you have MATLAB (R20XXx or newer) and the Control System Toolbox installed.
2. **Initialize Workspace:** 
   Navigate to `/matlab_scripts/` and run the initialization script to load kinematic parameters, dynamic assumptions, and MDREM gains:
   ```matlab
   run('init_params.m')
