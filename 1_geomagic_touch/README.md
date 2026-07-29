
# 🕹️ Geomagic Touch: Simulation & Experimental Validation

This module contains the high-fidelity **MATLAB/Simulink simulation environment** and the **experimental data validation** for the full system parameter identification (robot + payload) using the MDREM algorithm on the Geomagic Touch haptic device.

> ⚠️ **Implementation Note:** Due to laboratory intellectual property guidelines, the low-level C++ control firmware for the physical Geomagic Touch is not publicly shared. However, the complete simulation environment (which mimics the real system dynamics) and the experimental data logs proving hardware convergence are fully provided here.

---

## ⚙️ Technical Implementation Details

To keep this documentation highly scannable, detailed kinematic and dynamic modeling assumptions are collapsed below. Click to expand:

<details>
<summary><b>1. Kinematics & Denavit-Hartenberg (DH) Parameters</b></summary>
<br>

<div align="center">
  <img src="../Images/D-H configuration.jpg" alt="Description" width="500">
  <p><em>Figure 1.1. Denavit-Hartenberg configuration of the simplified 3 DoF Geomagic touch.</em></p>
</div>

The kinematics of the Geomagic Touch were modeled using standard DH conventions. Below are the parameters used to compute the Jacobian and regressor matrices:

| Joint | $a_i$ [m] | $d_i$ [m] | $\alpha_i$ [rad] | $q_i$ [rad] |
|-------|-----------|-----------|------------------|-------------|
| 1     | 0         | 0         | $\pi/2$          | $q_1^*$     |
| 2     | 0.145     | 0         | 0                | $q_2^*$     |
| 3     | 0.1738    | 0         | 0                | $q_3^*$     |

$*$ variable quantity

*(Note: Full derivations of transformation matrices are implemented in `matlab_scripts/init_params.m`)*
</details>

<details>
<summary><b>2. Dynamic Model & Geometric Simplifications</b></summary>
<br>
To achieve a reduced-order parameter formulation, several geometric simplifications were applied to the manipulator's links. 

* **Simulation:** The MDREM algorithm successfully identifies the full parameter vector `pi` (inertia, Coriolis, and gravity terms) within the ideal Simulink environment.
* **Hardware Validation:** In physical deployment, the geometric simplifications introduce structural modeling uncertainty. Consequently, the experimental validation successfully isolates and identifies the **gravitational dynamic parameters**, which dominate the low-speed static behavior of the manipulator.
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
