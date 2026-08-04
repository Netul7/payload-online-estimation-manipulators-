# 🦾 Franka Research 3 (FR3): Real-Time Payload Estimation

This module contains the **C++ hardware implementation** of the Modified Dynamic Regressor Extension and Mixing (MDREM) algorithm deployed on the 7-DOF Franka Research 3 collaborative robot. 

Unlike the Geomagic Touch implementation, this module focuses exclusively on the **unconstrained online estimation of the payload's inertial parameters** (mass, center of mass, and inertia tensor), without any geometrical simplifications, running in real-time within the manipulator's control loop.

---

## ⚙️ Technical Implementation & Robustness

To keep this overview scannable, mathematical parameterizations and details are collapsed below. Click to expand:

<details>
<summary><b>1. Payload Parameterization (The 10 Inertial Parameters)</b></summary>
<br>
The algorithm identifies the standard 10-dimensional inertial parameter vector of the unknown payload attached to the end-effector. The estimated vector $\theta$ is defined as:

` $\theta$ = [ m, m*cx, m*cy, m*cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz ]^T`

Where:
* `m`: Payload mass.
* `cx, cy, cz`: Mass moments.
* `Ixx ... Izz`: Elements of the inertia tensor at the center of mass.
* 
</details>

<details>
<summary><b>2. Kinematic Configuration</b></summary>
<br>

The kinematic configuration of the Franka research 3 robot follows the modified Denavit–Hartenberg convention or Craig’s convention. The modified D-H convention, in contrast to the original D-H convention, has the particularity of setting the *i*-th coordinate frame at the *i*-th joint. The image below shows kinematic configuration of the robot, while the table shows the  D-H parameters. 

<div align="center">
  <img src="../Images/dh-diagram-frankarobotics.png" alt="Description" width="500">
  <p><em>Figure 1.1. Denavit-Hartenberg configuration of the Franka Research 3.</em></p>
</div>

| Joint | $a_i$ [m] | $d_i$ [m] | $\alpha_i$ [rad] | $q_i$ [rad] |
|-------|-----------|-----------|------------------|-------------|
| 1     | 0         | 0.333     | 0                | $q_1^*$     |
| 2     | 0         | 0         | $-\frac{\pi}{2}$ | $q_2^*$     |
| 3     | 0         | 0.316     | $\frac{\pi}{2}$  | $q_3^*$     |
| 4     | 0.0825    | 0         | $\frac{\pi}{2}$  | $q_4^*$     |
| 5     | -0.0825   | 0.384     | $-\frac{\pi}{2}$ | $q_5^*$     |
| 6     | 0         | 0         | $\frac{\pi}{2}$  | $q_6^*$     |
| 7     | 0.088     | 0         | $\frac{\pi}{2}$  | $q_7^*$     |
| **Flange** | 0     | 0.107     | 0                | 0           |

$*$ variable quantity

The dynamic model of the FR3 robot is derived from the Newton-Euler (NE) formulation. In the NE formulation, each link is represented by ten inertial parameters. For this reason, in the linear parameterization model, the regressor results in a 7-by-70 matrix (7-by-80 if the payload is included), which makes it dificult to show here, so it is omitted.



</details>

<details>
<summary><b>3. Baseline Uncertainty & Bounded Convergence</b></summary>
<br>
A critical highlight of this experimental deployment is its robustness against unmodeled baseline dynamics. 

The known robot dynamic parameters (used to compensate the baseline internal dynamics) were derived from literature reported for the predecessor model, the **Franka Emika Panda**. Despite the physical and parametric discrepancies between the Panda and the FR3, the MDREM estimator did not diverge. Instead, it successfully rejected the structural uncertainty, driving the payload estimation error to a **bounded, practical neighborhood** of the true values.
</details>

---

## 📂 Software Architecture & Structure

The codebase is structured following standard C++ / [ROS 2] conventions for robotics applications:

* `/src/`: Core implementation of the MDREM algorithm, dynamic regressor computation, and hardware control loop.
* `/include/`: Header files, mathematical utilities (e.g., Eigen matrix definitions), and class declarations.
* `/config/`: YAML/JSON files containing estimator gains (adaptation rates, filter constants) and trajectory definitions.
* `/experimental_data/`: Sample `.csv` logs from physical payload lifting trials used to generate the convergence plots.

---

## 🚀 Getting Started (Building from Source)

### Prerequisites
* Linux Ubuntu [20.04 / 22.04]
* C++17 Compiler
* **Eigen 3** (for fast matrix algebra)
* **[libfranka / ROS 2 Humble/Galactic]** (Middleware and hardware interface)

### Compilation
To build the estimation node and control loop:

```bash
# 1. Clone the repository and navigate to this module
cd 2_franka_research_hardware

# 2. Build the project using CMake [or colcon build if using ROS]
mkdir build && cd build
cmake ..
make -j4
