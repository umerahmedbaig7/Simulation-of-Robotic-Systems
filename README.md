<div align="center">

# 🤖 Simulation of Robotic Systems
### MSc Robotics and Artificial Intelligence — Course Repository

[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3%2B-7B2FBE?style=for-the-badge&logo=openai&logoColor=white)](https://mujoco.org/)
[![NumPy](https://img.shields.io/badge/NumPy-1.21%2B-013243?style=for-the-badge&logo=numpy&logoColor=white)](https://numpy.org/)
[![SciPy](https://img.shields.io/badge/SciPy-1.7%2B-8CAAE6?style=for-the-badge&logo=scipy&logoColor=white)](https://scipy.org/)
[![Matplotlib](https://img.shields.io/badge/Matplotlib-3.4%2B-11557C?style=for-the-badge&logo=python&logoColor=white)](https://matplotlib.org/)
[![Status](https://img.shields.io/badge/Status-Complete-brightgreen?style=for-the-badge)]()
[![Field](https://img.shields.io/badge/Field-Robotics%20%26%20AI-blueviolet?style=for-the-badge&logo=ros&logoColor=white)]()

<br>

> *"Before you can control a robot, you must first teach a computer to believe in physics."*

<br>

**Author:** Umer Ahmed Baig Mughal <br>
**Programme:** MSc Robotics and Artificial Intelligence <br>
**Specialization:** Machine Learning · Computer Vision · Human-Robot Interaction · Autonomous Systems · Robotic Motion Control <br>
**Institution:** ITMO University — Faculty of Control Systems and Robotics

</div>

---

## 📋 Table of Contents

- [📖 About This Repository](#-about-this-repository)
- [🗂️ Repository Structure](#️-repository-structure)
- [🔬 Course Overview](#-course-overview)
- [🧪 Lab Summaries](#-lab-summaries)
  - [🔵 Lab 1 — Numerical Integration of a Damped Mass-Spring System](#-lab-1--numerical-integration-of-a-damped-mass-spring-system)
  - [🟠 Lab 2 — Passive RR Mechanism Simulation with Elastic Tendons in MuJoCo](#-lab-2--passive-rr-mechanism-simulation-with-elastic-tendons-in-mujoco)
  - [🟣 Lab 3 — Actuated RR Mechanism with PD Motor Control in MuJoCo](#-lab-3--actuated-rr-mechanism-with-pd-motor-control-in-mujoco)
  - [🟢 Lab 4 — Optimus Knee Mechanism: Closed-Loop Kinematics with Linear Actuator in MuJoCo](#-lab-4--optimus-knee-mechanism-closed-loop-kinematics-with-linear-actuator-in-mujoco)
- [🔗 Progressive Learning Pathway](#-progressive-learning-pathway)
- [⚙️ Simulation Framework — The Common Platform](#️-simulation-framework--the-common-platform)
- [🚀 Quick Start](#-quick-start)
- [📊 Results at a Glance](#-results-at-a-glance)
- [🧰 Tech Stack](#-tech-stack)
- [👤 Author](#-author)
- [📄 License](#-license)

---

## 📖 About This Repository

This repository contains the complete implementation of all four laboratory assignments from the **Simulation of Robotic Systems (SRS)** course, part of the MSc in Robotics and Artificial Intelligence at ITMO University. The labs form a deliberate and tightly connected progression — from first-principles mathematical simulation to advanced closed-loop kinematic mechanism modelling — covering the full spectrum of modern robotic system simulation techniques.

The course begins with **pure numerical methods** (Lab 1) to establish rigorous foundations in ODE integration and stability analysis, then transitions entirely into **MuJoCo-based 3D physics simulation** (Labs 2–4), advancing through passive dynamics, active PD control, and closed-loop kinematic mechanisms driven by a linear actuator.

All implementations are built from first principles. Every controller, actuator model, kinematic constraint, and data extraction pipeline is coded explicitly in Python without relying on robotics-specific toolboxes — ensuring deep understanding rather than surface-level toolchain usage.

### 🎯 What You Will Find Here

| 📁 Lab | 🏷️ Topic | 🧠 Core Concept | 🛠️ Key Tools |
|:------:|:--------:|:---------------:|:-------------:|
| Lab 1 | Numerical Integration | ODE Derivation · Euler · RK4 · Stability | NumPy · Matplotlib |
| Lab 2 | Passive RR Simulation | MuJoCo XML · Spatial Tendons · qpos Tracking | MuJoCo · Matplotlib |
| Lab 3 | Actuated RR Control | PD Controller · Sinusoidal Tracking · End-Effector | MuJoCo · NumPy · SciPy |
| Lab 4 | Closed-Loop Kinematics | Equality Constraints · Slide-Crank · Phase-Space | MuJoCo · NumPy · SciPy |

---

## 🗂️ Repository Structure

```
📦 Simulation-of-Robotic-Systems/
│
├── 📁 Lab_1/
│   ├── 📄 README.md                          # Lab 1 full documentation
│   ├── 📁 src/
│   │   ├── 🐍 Forward_Euler.py               # Explicit Euler integrator
│   │   ├── 🐍 Backward_Euler.py              # Implicit Euler integrator
│   │   ├── 🐍 K4_Euler.py                    # RK4 integrator
│   │   ├── 🐍 Forward_Euler_with_Eigenvalues.py
│   │   ├── 🐍 Backward_Euler_with_Eigenvalues.py
│   │   ├── 🐍 Runge_Kutta_with_Eigenvalues.py
│   │   ├── 🐍 Comparison.py                  # Side-by-side comparison
│   │   └── 🐍 Result.py                      # Metrics + overlaid plots
│   └── 📁 report/
│       └── 📋 Lab1_Report.pdf
│
├── 📁 Lab_2/
│   ├── 📄 README.md                          # Lab 2 full documentation
│   ├── 📁 src/
│   │   └── 🐍 Passive_RR_Mechanism.py        # Simulation + 3 plots
│   ├── 📁 model/
│   │   └── 🔧 MuJoCo_Model_Lab2.xml          # RR mechanism + tendons
│   ├── 📁 results/
│   │   └── 🖼️ Joint_Positions_Combined.png
│   └── 📁 report/
│       └── 📋 Lab2_Report.pdf
│
├── 📁 Lab_3/
│   ├── 📄 README.md                          # Lab 3 full documentation
│   ├── 📁 src/
│   │   └── 🐍 Actuated_RR_Mechanism.py       # PD controller + 4 plots
│   ├── 📁 model/
│   │   └── 🔧 MuJoCo_Model_with_Actuators.xml
│   ├── 📁 results/
│   │   ├── 🖼️ Joint_Positions_AB.png
│   │   ├── 🖼️ Joint_Velocities_AB.png
│   │   ├── 🖼️ End_Effector_Position.png
│   │   └── 🖼️ End_Effector_Pitch.png
│   └── 📁 report/
│       └── 📋 Lab3_Report.pdf
│
├── 📁 Lab_4/
│   ├── 📄 README.md                          # Lab 4 full documentation
│   ├── 📁 src/
│   │   └── 🐍 Optimus_Knee_Mechanism.py      # Ramp controller + 1 plot
│   ├── 📁 model/
│   │   └── 🔧 MuJoCo_Optimus_Knee_Model.xml
│   ├── 📁 results/
│   │   ├── 🖼️ Optimus_Knee_Model.jpg
│   │   └── 🖼️ Angle_Deviation_vs_Actuator_Position.png
│   └── 📁 report/
│       └── 📋 Lab4_Report.pdf
│
└── 📄 README.md                              # ← You are here
```

---

## 🔬 Course Overview

The **Simulation of Robotic Systems** course develops both the theoretical and practical tools for modelling, simulating, and controlling dynamic robotic mechanisms. The curriculum spans the mathematical foundations of dynamic system integration, the engineering practice of 3D physics-engine modelling, and the control theory needed to drive mechanisms along desired trajectories.

The four labs progress in a deliberate two-phase sequence:

**Phase 1 — Mathematical Foundation (Lab 1):** Before any 3D simulation, the fundamental problem of numerically integrating an ODE is tackled from scratch. The student derives the governing equation from Lagrangian mechanics, implements three integrators of increasing sophistication, and rigorously characterizes their stability and accuracy. This phase establishes the conceptual vocabulary — state-space representation, stability analysis, accuracy order — that underpins everything that follows.

**Phase 2 — Physics Engine Simulation (Labs 2–4):** With the numerical foundation in place, MuJoCo is introduced as the simulation platform. Labs 2 through 4 build on a single conceptual thread — an RR planar mechanism — progressing from passive dynamics to active PD control to a fundamentally new closed-loop mechanism topology. Each lab introduces exactly one new modelling concept, ensuring the incremental complexity remains tractable.

```
  ODE Foundation          Passive Physics        Active Control       Closed-Loop Mech.
  ─────────────           ──────────────         ──────────────       ──────────────────
  ┌─────────┐             ┌─────────┐            ┌─────────┐          ┌─────────┐
  │  LAB 1  │──────────►  │  LAB 2  │──────────► │  LAB 3  │────────► │  LAB 4  │
  └─────────┘             └─────────┘            └─────────┘          └─────────┘
  Integrators             MuJoCo XML             PD Controller        Equality Constraint
  Stability               Tendons                Motor Actuator       Slide-Crank Motor
  Eigenvalues             qpos Tracking          EE Kinematics        Phase-Space Output
  Pure Python             Passive Viewer         scipy Rotation       Closed Loop DOF
```

---

## 🧪 Lab Summaries

---

### 🔵 Lab 1 — Numerical Integration of a Damped Mass-Spring System

<div align="center">

[![Lab1](https://img.shields.io/badge/Lab%201-Numerical%20Integration-0078D7?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Methods-Euler%20%7C%20RK4-lightblue?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-Stability%20Analysis-lightblue?style=flat-square)]()
[![Scripts](https://img.shields.io/badge/Scripts-8%20Python%20files-lightblue?style=flat-square)]()

</div>

#### 📌 Task Description

> Given a **damped mass-spring system** (Variant 2: $m=0.4$ kg, $k=4.4$ N/m, $b=0.025$ N·s/m), derive the equation of motion from first principles, reformulate it as a first-order state-space system, and simulate it using three numerical integrators. Rigorously compare their accuracy and stability over short and long simulation horizons.

The lab answers the foundational question of numerical simulation: *"Given a dynamic system's governing ODE, how do different integration strategies affect the accuracy, stability, and physical fidelity of the computed solution over time?"*

**What the task requires:**
- Derive the governing ODE using the **Euler-Lagrange formalism** from kinetic and potential energy expressions.
- Reformulate the scalar second-order ODE as a **first-order vector state-space system** $\dot{s} = As$, where $s = [x,\; \dot{x}]^T$.
- Implement the **Explicit (Forward) Euler** method and characterize its conditional stability — unstable for $h > h_\text{crit} \approx 0.006$ s.
- Implement the **Implicit (Backward) Euler** method using a **fixed-point iteration loop** converging to tolerance $10^{-8}$, and understand its unconditional A-stability.
- Implement the **Runge-Kutta 4th Order (RK4)** method as the preferred choice for physical simulation combining high accuracy with practical stability.
- Construct the **discrete-time update matrix** $A_d(h)$ and sweep the **eigenvalue norm** $|\lambda_\text{max}|$ over a range of step sizes to map the stability boundary of each method numerically.
- Compare all three integrators quantitatively over $t_f = 10$ s and $t_f = 30$ s, measuring amplitude growth, terminal error, and physical fidelity.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| ⚡ Lagrangian Mechanics | ODE derived from $\mathcal{L} = K - U$ via Euler-Lagrange equation with non-conservative damping |
| 📐 State-Space Form | $\dot{s}=As$, where $A = [[0, 1], [-k/m, -b/m]]$ — enables single-step vector integration |
| 🔵 Forward Euler | $s_{n+1} = s_n + h \cdot f(s_n)$ — explicit, 1st order, conditionally stable |
| 🟠 Backward Euler | $s_{n+1} = s_n + h \cdot f(s_{n+1})$ — implicit, 1st order, unconditionally A-stable |
| 🟣 RK4 | Four-slope weighted average — 4th order, preferred for physical accuracy |
| 📊 Eigenvalue Stability | $\lambda_\text{max}(A_d(h)) \leq 1$ required for numerical stability; swept vs $h$ |
| ⚠️ Stability Boundary | Forward Euler: $h_\text{crit} \approx 0.006$ s; $\lambda = 1.000237$ at $h = 0.01$ s |

#### 📤 Key Numerical Results

```
System:  m=0.4 kg,  k=4.4 N/m,  b=0.025 N·s/m,  x₀=0.46 m,  ẋ₀=0 m/s
         ωₙ = 3.3166 rad/s,  ζ = 0.00942  →  underdamped (slowly decaying)

                      t_f = 30 s,  h = 0.01 s
┌─────────────────┬──────────────────┬──────────────────────────────┐
│ Integrator      │ Max Amplitude (m)│ Final Error vs Backward Euler│
├─────────────────┼──────────────────┼──────────────────────────────┤
│ Backward Euler  │ 0.4600           │ — (reference baseline)       │
│ Forward Euler   │ 0.9239     grows │ 2.6219                       │
│ RK4             │ 0.4600     decays│ 0.4183                       │
└─────────────────┴──────────────────┴──────────────────────────────┘
Backward Euler fixed-point convergence: 6 iterations to ε < 10⁻⁸ ✓
```

📂 **[→ View Lab 1 Full Documentation](https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems/blob/main/Lab_1/Readme.md)**

---

### 🟠 Lab 2 — Passive RR Mechanism Simulation with Elastic Tendons in MuJoCo

<div align="center">

[![Lab2](https://img.shields.io/badge/Lab%202-Passive%20RR%20Simulation-E85D04?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Engine-MuJoCo%20XML-orange?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-Joint%20Position%20Plots-orange?style=flat-square)]()
[![Scripts](https://img.shields.io/badge/Scripts-1%20Python%20%2B%201%20XML-orange?style=flat-square)]()

</div>

#### 📌 Task Description

> Build a **passive RR (Revolute-Revolute) mechanism** of open kinematics in MuJoCo XML with two elastic crossed cables routed over cylindrical pulleys. Simulate it in real-time and track both joint positions over 5-second and 15-second runs, generating position vs time plots.

This lab marks the transition from analytical numerical integration to **full 3D physics-engine simulation**. The student must model every physical component — bodies, joints, geometries, sites, pulleys, and tendons — explicitly in MuJoCo's XML format, then write a Python script to drive and record the simulation.

**What the task requires:**
- Build a **hierarchical body tree** in MuJoCo XML representing the two-link serial RR chain with three link lengths ($a=0.045$ m, $b=0.039$ m, $c=0.055$ m) and two revolute joints rotating about the $y$-axis with ±90° hard limits.
- Place **named sites** (`s1`–`s8`, `c1`) at precise geometric locations on each link to serve as tendon anchor and routing waypoints.
- Model cylindrical **pulleys** at joint 1 (radius $R_1=0.01$ m) and joint 2 (radius $R_2=0.01$ m) using `<geom type="cylinder">`.
- Add **two crossed spatial elastic tendons** (stiffness=10 N/m each) using `<tendon><spatial>`, routing each tendon over both pulleys to create opposing restoring forces.
- Write a Python script using `mujoco` and `mujoco.viewer` APIs to step the simulation in real-time, collect `qpos` from both joints at every timestep, and generate 3 position plots per simulation run.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| 🔧 MuJoCo XML | Hierarchical `<worldbody>` tree encoding bodies, joints, geometries, and sites |
| 📌 Named Sites | Anchor points for tendon routing — `s1`–`s8` + shared intermediate `c1` |
| 🔄 Spatial Tendons | Piecewise-linear cables wrapping over pulley side-sites; force = $k \cdot \Delta L$ |
| ⚙️ Joint Defaults | `springdamper="1 100"` — 1 N·m/rad stiffness, 100 N·m·s/rad damping at each joint |
| 📈 qpos Tracking | `data.qpos[joint_dofadr]` — generalized joint positions recorded every simulation step |
| 🔁 Passive Viewer | `mujoco.viewer.launch_passive()` — real-time render without blocking the data loop |
| 🎛️ Tendon Routing | Tendon 1 (red): s1→Pulley1→c1→Pulley2→s8; Tendon 2 (green): s2→Pulley1→c1→Pulley2→s7 |

#### 📤 Key Simulation Results

```
System:  2-DOF passive RR, 2 crossed tendons, timestep=0.002 s

         At t_f = 5 s                    At t_f = 15 s
┌────────────┬──────────────────┐  ┌────────────┬──────────────────────┐
│ Joint      │ Range (rad)      │  │ Joint      │ Steady State (rad)   │
├────────────┼──────────────────┤  ├────────────┼──────────────────────┤
│ J1 (one)   │ 0.00 → 1.75      │  │ J1 (one)   │ ~1.10 – 1.40         │
│ J2 (two)   │ −0.50 → 1.75     │  │ J2 (two)   │ ~0.40 – 0.60         │
└────────────┴──────────────────┘  └────────────┴──────────────────────┘
Both joints exhibit damped oscillation converging to stable periodic motion by ~5–7 s
```

📂 **[→ View Lab 2 Full Documentation](https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems/blob/main/Lab_2/Readme.md)**

---

### 🟣 Lab 3 — Actuated RR Mechanism with PD Motor Control in MuJoCo

<div align="center">

[![Lab3](https://img.shields.io/badge/Lab%203-PD%20Motor%20Control-7B2FBE?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Control-PD%20Controller-purple?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-4%20Time--Series%20Plots-purple?style=flat-square)]()
[![Scripts](https://img.shields.io/badge/Scripts-1%20Python%20%2B%201%20XML-purple?style=flat-square)]()

</div>

#### 📌 Task Description

> Extend the passive Lab 2 RR mechanism by **adding a position actuator at joint B** and implementing a **PD controller** to track a sinusoidal reference trajectory. Additionally, track and visualize the **end-effector position and pitch orientation** throughout the 10-second simulation.

This lab introduces the complete sense-plan-act loop: at every simulation step, the controller reads joint state, computes a corrective torque, writes it to the actuator, and the physics engine propagates the effect forward. The student must simultaneously track task-space kinematics, requiring rotation matrix extraction and Euler angle computation.

**What the task requires:**
- Modify the Lab 2 XML model: rename joints to `A`/`B`, remove joint spring-damper from defaults, and add a `<position>` actuator (`Motor B`) at joint B with `ctrllimited="true"`, `ctrlrange="-1 1"`, `gear="1"`.
- Implement a **PD controller**: `control_signal = KP × (target − q_B) − KD × q̇_B`, writing to `data.ctrl[actuator_id]` at each step. Gains: KP = KD = 0.005.
- Generate a **sinusoidal reference trajectory**: $q_{B,\text{target}}(t) = 1.073 \cdot \sin(1.564 \cdot t)$ rad.
- Extract **end-effector position** (x, y, z) from `data.geom_xpos` for the `right_wall` geometry.
- Extract **end-effector pitch** by reshaping `data.geom_xmat` to a $3\times3$ rotation matrix, converting via `scipy.spatial.transform.Rotation.from_matrix()`, and extracting `euler_angles[1]`.
- Generate **4 time-series plots**: joint positions A&B, joint velocities A&B, end-effector x/y/z position, end-effector pitch orientation.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| ⚡ Position Actuator | `<position>` element drives joint B toward a target angle via `data.ctrl` |
| 🎮 PD Control Law | `u = KP·e − KD·ė` — proportional error correction with derivative damping |
| 🌊 Sinusoidal Reference | $1.073 \cdot \sin(1.564 \cdot t)$ — amplitude 1.073 rad, angular frequency 1.564 rad/s |
| 🔗 End-Effector Tracking | `geom_xpos` for position; `geom_xmat` → 3×3 → scipy Rotation → Euler `'xyz'` |
| 📐 Pitch Extraction | `euler_angles[1]` — y-axis rotation of `right_wall` in degrees |
| 🔄 Joint Coupling | Joint A remains passive; its uncontrolled motion dominates end-effector trajectory |
| 🧰 ctrl Access | `mj_name2id(m, mjOBJ_ACTUATOR, name)` → index into `data.ctrl[]` |

#### 📤 Key Simulation Results

```
System:  RR + Motor B, PD gains KP=KD=0.005, sinusoidal target, timestep=0.002 s

┌──────────────────────────┬─────────────────┬──────────────────────────────────┐
│ Channel                  │ Range           │ Behaviour                        │
├──────────────────────────┼─────────────────┼──────────────────────────────────┤
│ Joint A position (rad)   │ 0.00 – 1.75     │ Passive — large transient, decays│
│ Joint B position (rad)   │ ≈ 0             │ Actuated — PD holds near zero    │
│ Joint A velocity (rad/s) │ −5 to +17       │ High transient, settles ±3 rad/s │
│ Joint B velocity (rad/s) │ ≈ 0             │ Near-zero — controller effective │
│ EE x-position (m)        │ 0.04 – 0.16     │ Oscillates with Joint A          │
│ EE y-position (m)        │ 0               │ Always zero (x–z plane motion)   │
│ EE z-position (m)        │ 0.20 – 0.30     │ Oscillates with Joint A          │
│ EE pitch (°)             │ 0 – 85          │ Tracks Joint A angular motion    │
└──────────────────────────┴─────────────────┴──────────────────────────────────┘
```

📂 **[→ View Lab 3 Full Documentation](https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems/blob/main/Lab_3/Readme.md)**

---

### 🟢 Lab 4 — Optimus Knee Mechanism: Closed-Loop Kinematics with Linear Actuator in MuJoCo

<div align="center">

[![Lab4](https://img.shields.io/badge/Lab%204-Closed--Loop%20Kinematics-2E7D32?style=flat-square&logo=python&logoColor=white)]()
[![Method](https://img.shields.io/badge/Mechanism-Four--Bar%20Linkage-green?style=flat-square)]()
[![Output](https://img.shields.io/badge/Output-Phase--Space%20Plot-green?style=flat-square)]()
[![Scripts](https://img.shields.io/badge/Scripts-1%20Python%20%2B%201%20XML-green?style=flat-square)]()

</div>

#### 📌 Task Description

> Model the **Optimus Knee Mechanism** (Variant 2) — a closed-loop four-bar linkage that replicates knee joint motion — in MuJoCo XML. Set it in motion using a **linear slide-crank actuator** driven by a ramp controller. Plot the rotation of the crank link OA as a function of the actuator's linear displacement, revealing the mechanism's kinematic transmission curve.

This lab introduces the most structurally complex MuJoCo model of the course — a closed-loop mechanism that cannot be expressed as a single body tree and requires explicit kinematic loop closure via an equality constraint. The output shifts from time-series plots to a **phase-space kinematic curve**, characterizing the mechanism's full range of motion in a single simulation run.

**What the task requires:**
- Build **two separate body hierarchies** in `<worldbody>` — the crank-side chain (joints O, A) and the rocker-side chain (joints C, B) — anchored at their respective fixed ground pivots.
- Close the kinematic loop using `<equality><connect site1="s1" site2="s2">`, constraining the tip of the coupler link AB (site `s1`) to always coincide with the top of the rocker CB (site `s2`).
- Configure a **slide-crank `<motor>` actuator** using `slidersite="LA_site"`, `cranksite="s2"`, and `cranklength="0.295"` (L5) — driving the mechanism through a connecting rod geometry.
- Implement a **ramp controller** that increments `data.ctrl[0]` by `CTRL_INCREMENT=0.0005` per step from 0 to the XML-defined maximum of 0.5, sweeping the mechanism through its full kinematic range.
- Track the crank OA rotation by extracting `data.geom_xmat` for `Length_1`, reshaping to $3\times3$, converting via scipy Rotation, and recording the y-axis deviation from the initial angle.
- Track actuator displacement via `data.actuator_length[0]` — the physical length of the slide-crank connecting rod.
- Generate a **single phase-space plot**: crank OA angle deviation (−10° to −90°) vs actuator length deviation.

#### 🔑 Key Concepts

| Concept | Description |
|---------|-------------|
| 🔄 Closed-Loop Kinematics | Two body trees + `<connect>` constraint = single closed four-bar linkage |
| ⛓️ Equality Constraint | `<connect site1="s1" site2="s2">` — holonomic loop closure via MuJoCo constraint solver |
| 🔩 Slide-Crank Motor | `slidersite` + `cranksite` + `cranklength` — converts linear → rotational motion |
| 📈 Ramp Controller | Open-loop monotonic increment: `ctrl += CTRL_INCREMENT` each step until `ctrlrange` max |
| 📏 Actuator Length | `data.actuator_length[0]` — physical connecting rod length, not a generalized coordinate |
| 📊 Phase-Space Output | Angle deviation vs displacement deviation — kinematic transmission map, not time-series |
| 🧩 1-DOF Mechanism | Closed loop reduces system to 1 DOF — full state determined by single actuator value |

#### 📤 Key Simulation Results

```
System:  Optimus Knee (Variant 2), slide-crank motor, CTRL_INCREMENT=0.0005, timestep=0.001 s
         L1=0.059m, L2=0.0767m, L3=0.0885m, L4=0.059m, L5=0.295m (cranklength)

┌──────────────────────────────┬──────────────────────────────────────────┐
│ Output Quantity              │ Result                                   │
├──────────────────────────────┼──────────────────────────────────────────┤
│ Actuator deviation range (m) │ ≈ −0.04 to +0.01                         │
│ Crank OA angle deviation (°) │ −10° → −90°  (≈ 80° total sweep)         │
│ Curve shape                  │ Non-linear — high sensitivity near start │
│ Ramp completion time         │ ~1 s (1000 steps at 0.001 s/step)        │
│ Total simulation duration    │ 10 s (mechanism holds at max for 9 s)    │
└──────────────────────────────┴──────────────────────────────────────────┘
```

📂 **[→ View Lab 4 Full Documentation](https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems/blob/main/Lab_4/Readme.md)**

---

## 🔗 Progressive Learning Pathway

The four labs form an interlocking progression where each builds explicitly on the foundations of the previous one. The dependency chain is not merely thematic — specific concepts, APIs, and modelling patterns introduced in each lab are carried forward and extended in the next.

```
╔═════════════════════════════════════════════════════════════════════════════════════════╗
║                         SIMULATION OF ROBOTIC SYSTEMS — PROGRESSION                     ║
╠══════════════════╦═══════════════════╦═══════════════════╦══════════════════════════════╣
║  🔵 LAB 1       ║  🟠 LAB 2         ║  🟣 LAB 3         ║  🟢 LAB 4                   ║
║  Numerical       ║  Passive RR       ║  Actuated RR      ║  Closed-Loop Knee            ║
║  Integration     ║  Simulation       ║  PD Control       ║  Mechanism                   ║
╠══════════════════╬═══════════════════╬═══════════════════╬══════════════════════════════╣
║  ODE + state-    ║  MuJoCo XML +     ║  Position         ║  Equality constraint         ║
║  space form      ║  tendon routing   ║  actuator + PD    ║  + slide-crank motor         ║
║  3 integrators   ║  passive viewer   ║  scipy rotation   ║  phase-space output          ║
╠══════════════════╬═══════════════════╬═══════════════════╬══════════════════════════════╣
║  No MuJoCo       ║  No control       ║  Extends Lab 2    ║  New topology entirely       ║
║  8 scripts       ║  XML model        ║  XML model        ║  2 body trees + constraint   ║
║  NumPy only      ║  + 1 Python       ║  + 1 Python       ║  + 1 Python                  ║
╠══════════════════╬═══════════════════╬═══════════════════╬══════════════════════════════╣
║  Timestep        ║  Timestep         ║  Timestep         ║  Timestep                    ║
║  h = 0.01 s      ║  0.002 s (Python) ║  0.002 s (Python) ║  0.001 s (XML)               ║
║  (manual)        ║  time.sleep()     ║  time.sleep()     ║  model.opt.timestep          ║
╠══════════════════╬═══════════════════╬═══════════════════╬══════════════════════════════╣
║  Output used     ║  Model reused     ║  Extends Lab 2    ║  New mechanism               ║
║  as concept      ║  in Lab 3         ║  XML + tendons    ║  Introduces closed-loop      ║
║  foundation      ║                   ║  + adds Motor B   ║  DOF-1 kinematics            ║
╚══════════════════╩═══════════════════╩═══════════════════╩══════════════════════════════╝
```

**Dependency chain:**
- 🔵 **Lab 1** establishes the vocabulary of state-space dynamics, timestep sensitivity, and numerical stability — concepts directly relevant to every MuJoCo simulation that follows.
- 🟠 **Lab 2** introduces MuJoCo XML modelling, the spatial tendon framework, and the passive viewer loop pattern. The RR mechanism XML and all tendon infrastructure are directly inherited by Lab 3.
- 🟣 **Lab 3** extends the Lab 2 XML with one actuator, replaces passive joint dynamics with a PD feedback loop, and introduces `scipy.spatial.transform` for rotation extraction — a technique reused in Lab 4.
- 🟢 **Lab 4** introduces a completely new mechanism topology (closed-loop) while reusing the scipy rotation extraction pipeline from Lab 3 and the passive viewer loop pattern from Lab 2.

---

## ⚙️ Simulation Framework — The Common Platform

### 🔵 Lab 1 — Pure Numerical Framework

Lab 1 operates entirely in Python with NumPy and Matplotlib. The common platform is the **state-space ODE framework**:

```
State vector:   s = [x, ẋ]ᵀ
Dynamics:       ṡ = A·s,     A = [ 0     1   ]
                                   [-k/m  -b/m ]
Integrators:    Forward Euler  →  s_{n+1} = s_n + h·f(s_n)
                Backward Euler →  s_{n+1} = fixed-point iteration to ε < 10⁻⁸
                RK4            →  s_{n+1} = weighted 4-slope average
```

The system parameters for the primary variant (Variant 2) used in the individual integrator scripts:

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.4 | kg |
| Spring constant | $k$ | 4.4 | N/m |
| Damping coefficient | $b$ | 0.025 | N·s/m |
| Initial displacement | $x_0$ | 0.46 | m |
| Initial velocity | $\dot{x}_0$ | 0.0 | m/s |
| Natural frequency | $\omega_n$ | 3.3166 | rad/s |
| Damping ratio | $\zeta$ | 0.00942 | — |

> 💡 A second parameter set ($m=0.9$, $k=11.8$, $b=0.015$) is used exclusively in the three eigenvalue scripts to produce a broader stability illustration. These are intentionally different from the primary variant and their outputs should not be compared directly with those of the individual integrator scripts.

---

### 🟠🟣🟢 Labs 2–4 — MuJoCo Physics Engine Framework

Labs 2, 3, and 4 all operate within **MuJoCo 2.3+**, sharing the same fundamental simulation loop pattern, API conventions, and file organization. The common workflow across all three labs is:

```python
# 1. Load model
model = mujoco.MjModel.from_xml_path('path/to/model.xml')
data  = mujoco.MjData(model)

# 2. Launch passive viewer (non-blocking)
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running() and time.time() - start_time < DURATION:
        step_start = time.time()

        # 3. Read state  →  4. Compute/apply control  →  5. Record data
        #    data.qpos, data.qvel, data.geom_xpos, data.geom_xmat, data.actuator_length

        mujoco.mj_step(model, data)   # 6. Advance simulation
        viewer.sync()                  # 7. Update render
        time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))

# 8. Plot collected data with Matplotlib
```

**MuJoCo XML elements introduced progressively across Labs 2–4:**

| XML Element | Introduced | Purpose |
|-------------|-----------|---------|
| `<worldbody>`, `<body>`, `<joint>`, `<geom>` | Lab 2 | Body hierarchy, joints, geometries |
| `<site>` | Lab 2 | Named anchor points for tendons and constraints |
| `<tendon><spatial>` | Lab 2 | Elastic cable routing over pulleys |
| `<defaults><joint springdamper>` | Lab 2 | Global joint spring-damper |
| `<actuator><position>` | Lab 3 | Position-controlled motor at a joint |
| `data.ctrl`, `data.geom_xmat` | Lab 3 | Actuator control + rotation matrix |
| `<equality><connect>` | Lab 4 | Kinematic loop closure constraint |
| `<actuator><motor slidersite cranksite>` | Lab 4 | Slide-crank linear actuator |
| `data.actuator_length` | Lab 4 | Physical actuator rod length |

---

## 🚀 Quick Start

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems.git
cd Simulation-of-Robotic-Systems
```

### 2️⃣ Install Dependencies

**For Lab 1 only (no MuJoCo required):**
```bash
pip install numpy matplotlib
```

**For Labs 2–4 (full stack):**
```bash
pip install mujoco matplotlib numpy scipy
```

> 📌 MuJoCo 2.3+ includes its own Python bindings and passive viewer. No separate viewer installation is needed. A working display environment is required (X server on Linux, native on Windows/macOS). For a clean virtual environment:
> ```bash
> python -m venv srs_env
> source srs_env/bin/activate       # Windows: srs_env\Scripts\activate
> pip install mujoco matplotlib numpy scipy
> ```

### 3️⃣ Update XML Paths (Labs 2–4)

Each MuJoCo lab script contains a hardcoded absolute path to its XML model file. Before running, open the script and update the path to your local copy:

```python
# Lab 2 — Passive_RR_Mechanism.py
m = mujoco.MjModel.from_xml_path('path/to/MuJoCo_Model_Lab2.xml')

# Lab 3 — Actuated_RR_Mechanism.py
m = mujoco.MjModel.from_xml_path('path/to/MuJoCo_Model_with_Actuators.xml')

# Lab 4 — Optimus_Knee_Mechanism.py
MODEL_NAME = 'path/to/MuJoCo_Optimus_Knee_Model.xml'
```

### 4️⃣ Run Each Lab

```bash
# 🔵 Lab 1 — Run individual integrators (10 s simulation each)
python Lab_1/src/Forward_Euler.py
python Lab_1/src/Backward_Euler.py
python Lab_1/src/K4_Euler.py

# 🔵 Lab 1 — Run stability analysis (eigenvalue norm plots)
python Lab_1/src/Forward_Euler_with_Eigenvalues.py
python Lab_1/src/Backward_Euler_with_Eigenvalues.py
python Lab_1/src/Runge_Kutta_with_Eigenvalues.py

# 🔵 Lab 1 — Run full comparison and metrics (30 s run, printed results)
python Lab_1/src/Comparison.py
python Lab_1/src/Result.py

# 🟠 Lab 2 — Passive RR simulation (opens MuJoCo viewer + 3 plots)
python Lab_2/src/Passive_RR_Mechanism.py

# 🟣 Lab 3 — Actuated RR simulation (opens MuJoCo viewer + 4 plots)
python Lab_3/src/Actuated_RR_Mechanism.py

# 🟢 Lab 4 — Optimus Knee simulation (opens MuJoCo viewer + 1 phase-space plot)
python Lab_4/src/Optimus_Knee_Mechanism.py
```

> 📌 For Labs 2–4: keep the MuJoCo viewer window open for the full simulation duration. Closing it early will truncate data collection and produce incomplete plots.

---

## 📊 Results at a Glance

### 🔵 Lab 1 — Integrator Stability and Accuracy

| Integrator | Stability Type | $\lambda_\text{max}$ at $h=0.01$ s | Max Amplitude at $t=30$ s | Physical Fidelity |
|:----------:|:--------------:|:-----------------------------------:|:------------------------:|:----------------:|
| Forward Euler | Conditionally stable | 1.000237 ❌ | 0.9239 m (grows) | Poor — energy injection |
| Backward Euler | A-stable (unconditional) | < 1.0 ✅ | 0.4600 m (decays fast) | Over-damped |
| RK4 | Conditionally stable | ≈ 1.0 ✅ | 0.4600 m (physical decay) | Excellent ✅ |

### 🟠 Lab 2 — Passive RR Joint Convergence

| Simulation | Joint | Initial Amplitude | Steady-State Range | Convergence Time |
|:----------:|:-----:|:-----------------:|:-----------------:|:----------------:|
| 5 s run | J1 | ~0 → 1.75 rad | Not yet converged | > 5 s |
| 5 s run | J2 | −0.50 → 1.75 rad | Not yet converged | > 5 s |
| 15 s run | J1 | ~0 → 1.75 rad | 1.10 – 1.40 rad | ~5–7 s |
| 15 s run | J2 | −0.50 → 1.75 rad | 0.40 – 0.60 rad | ~5–7 s |

### 🟣 Lab 3 — PD Control Performance Summary

| Channel | Range | Controller Effect |
|:-------:|:-----:|:-----------------:|
| Joint A (passive) | 0 – 1.75 rad | Uncontrolled — large transient, decays slowly |
| Joint B (actuated) | ≈ 0 rad | Held near zero by PD — low-authority gains |
| EE x-position | 0.04 – 0.16 m | Driven by uncontrolled Joint A |
| EE y-position | 0 m | Always zero — x–z plane motion only |
| EE pitch | 0 – 85° | Tracks Joint A angular displacement |

### 🟢 Lab 4 — Kinematic Transmission Curve

| Quantity | Value | Interpretation |
|:--------:|:-----:|:--------------:|
| Actuator deviation range | −0.04 to +0.01 m | Full linear stroke of slide-crank |
| Crank OA angle range | −10° to −90° | ~80° total angular sweep |
| Curve shape | Non-linear | Mechanical advantage varies with configuration |
| Ramp completion | ~1 s of 10 s total | Full kinematic range swept in first 10% of simulation |
| Loop closure quality | Smooth, no oscillation | Constraint solver maintains s1=s2 throughout |

---

## 🧰 Tech Stack

<div align="center">

| 🛠️ Tool | 🔖 Version | 🎯 Role in This Course | 🧪 Used In |
|:-------:|:---------:|:---------------------:|:----------:|
| ![Python](https://img.shields.io/badge/-Python-3776AB?logo=python&logoColor=white) | 3.8+ | Core language — all labs | All |
| ![MuJoCo](https://img.shields.io/badge/-MuJoCo-7B2FBE?logo=openai&logoColor=white) | ≥ 2.3 | Physics engine, XML model loading, constraint solver, passive viewer | Labs 2–4 |
| ![NumPy](https://img.shields.io/badge/-NumPy-013243?logo=numpy&logoColor=white) | ≥ 1.21 | ODE integration, matrix ops, eigenvalue computation, rotation reshape | All |
| ![SciPy](https://img.shields.io/badge/-SciPy-8CAAE6?logo=scipy&logoColor=white) | ≥ 1.7 | `Rotation.from_matrix()` — geom_xmat → Euler angle extraction | Labs 3–4 |
| ![Matplotlib](https://img.shields.io/badge/-Matplotlib-11557C?logo=python&logoColor=white) | ≥ 3.4 | Time-series plots, stability curves, phase-space plots | All |

</div>

**No robotics toolboxes. No MATLAB. No ROS.** Every algorithm — from the Runge-Kutta integrator to the closed-loop constraint model — is implemented explicitly in Python, ensuring depth of understanding at every level of the simulation stack.

---

## 👤 Author

<div align="center">

### Umer Ahmed Baig Mughal

🎓 **MSc Robotics and Artificial Intelligence** — ITMO University <br>
🏛️ *Faculty of Control Systems and Robotics* <br>
🔬 *Specialization: Machine Learning · Computer Vision · Human-Robot Interaction · Autonomous Systems · Robotic Motion Control*

[![GitHub](https://img.shields.io/badge/GitHub-umerahmedbaig7-181717?style=for-the-badge&logo=github)](https://github.com/umerahmedbaig7)

</div>

---

## 📄 License

This repository is intended for **academic and research use**. All work was developed as part of the *Simulation of Robotic Systems* course within the MSc Robotics and Artificial Intelligence program at ITMO University. Redistribution, modification, and use in derivative academic work are permitted with appropriate attribution to the original author.

---

<div align="center">

*Simulation of Robotic Systems — MSc Robotics and Artificial Intelligence | ITMO University*

⭐ *If this repository helped you understand robotic system simulation, consider giving it a star!* ⭐

</div>

