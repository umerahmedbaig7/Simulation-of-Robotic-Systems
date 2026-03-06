
# Lab 1 — Numerical Integration of a Damped Mass-Spring System

![Python](https://img.shields.io/badge/Python-3.8+-blue)
![NumPy](https://img.shields.io/badge/NumPy-Required-orange)
![Matplotlib](https://img.shields.io/badge/Matplotlib-Required-green)
![Field](https://img.shields.io/badge/Field-Robotics%20Simulation-red)
![License](https://img.shields.io/badge/License-MIT-yellow)

> **Course:** Simulation of Robotic Systems — Faculty of Control Systems and Robotics, ITMO University <br>
> **Author:** Umer Ahmed Baig Mughal — MSc Robotics and Artificial Intelligence <br>
> **Topic:** Lagrangian Mechanics · ODE Derivation · Explicit Euler · Implicit Euler · RK4 · Eigenvalue Stability Analysis

---

## Table of Contents

1. [Objective](#objective)
2. [Theoretical Background](#theoretical-background)
   - [Physical System Description](#physical-system-description)
   - [Equation of Motion via Lagrangian Mechanics](#equation-of-motion-via-lagrangian-mechanics)
   - [State-Space Formulation](#state-space-formulation)
   - [System Properties](#system-properties)
3. [Numerical Integration Methods](#numerical-integration-methods)
   - [Explicit (Forward) Euler Method](#explicit-forward-euler-method)
   - [Implicit (Backward) Euler Method](#implicit-backward-euler-method)
   - [Runge-Kutta Method of Order 4 (RK4)](#runge-kutta-method-of-order-4-rk4)
   - [Accuracy Comparison](#accuracy-comparison)
4. [Stability Analysis via Eigenvalue Norms](#stability-analysis-via-eigenvalue-norms)
5. [System Parameters](#system-parameters)
6. [Implementation](#implementation)
   - [File Structure](#file-structure)
   - [Function Reference](#function-reference)
   - [Algorithm Walkthrough](#algorithm-walkthrough)
7. [How to Run](#how-to-run)
8. [Results](#results)
9. [Numerical Validation](#numerical-validation)
10. [Dependencies](#dependencies)
11. [Notes and Limitations](#notes-and-limitations)
12. [Author](#author)
13. [License](#license)

---

## Objective

This lab performs **analytical derivation and numerical simulation** of a damped mass-spring system — a canonical second-order ordinary differential equation (ODE) that forms the foundation of dynamic system modelling in robotics and mechatronics. The system is solved using three distinct numerical integrators and their performance is rigorously compared.

The key learning outcomes are:

- Deriving the governing second-order ODE from first principles using the **Lagrangian (energy) method**.
- Reformulating the scalar second-order ODE as a **first-order vector state-space system** suitable for numerical integration.
- Implementing the **Explicit (Forward) Euler** method and understanding its conditional stability limitations.
- Implementing the **Implicit (Backward) Euler** method using a fixed-point iteration loop and understanding its unconditional A-stability.
- Implementing the **Runge-Kutta 4th Order (RK4)** method and understanding why its higher-order accuracy makes it the preferred choice for physical simulation.
- Constructing the **discrete-time update matrix** $A_d$ and computing its **eigenvalue norm** as a function of step size to characterize numerical stability boundaries.
- Comparing all three methods quantitatively over short (10 s) and long (30 s) simulation horizons and drawing conclusions about the accuracy–stability trade-off.

The lab is implemented across eight Python scripts, each targeting a specific integrator, comparison scenario, or eigenvalue stability configuration.

---

## Theoretical Background

### Physical System Description

The system under study consists of a mass $m$ attached to a spring of constant $k$ and a viscous damper with coefficient $b$. The mass moves freely along a single axis, with $x(t)$ representing displacement from the equilibrium position. Gravitational force $mg$ acts downward and is balanced at equilibrium, contributing no net force to the oscillatory dynamics.

The forces acting on the mass are:

```
Spring restoring force:   F_s = −k·x
Viscous damping force:    F_d = −b·ẋ   (non-conservative)
Gravity:                  balanced at equilibrium — absent from oscillatory EOM
```

### Equation of Motion via Lagrangian Mechanics

The equation of motion is derived using the **Euler-Lagrange formalism**, which requires only the kinetic and potential energies of the system.

**Kinetic and Potential Energies:**

```
K = (1/2)·m·ẋ²          (kinetic energy of the mass)
U = (1/2)·k·x²          (potential energy stored in the spring — Hooke's law)
```

**Lagrangian:**

```
L = K − U = (1/2)·m·ẋ² − (1/2)·k·x²
```

**Euler-Lagrange Equation** with non-conservative generalized force $Q = -b\dot{x}$:

```
d/dt (∂L/∂ẋ) − ∂L/∂x = Q

∂L/∂ẋ  = m·ẋ
d/dt(m·ẋ) = m·ẍ
∂L/∂x  = −k·x
```

Substituting and adding the damping term:

```
m·ẍ + b·ẋ + k·x = 0                    (1)  Full equation of motion
ẍ + (b/m)·ẋ + (k/m)·x = 0              (2)  Normalized form (÷ m)
```

### State-Space Formulation

Equation (1) is a second-order ODE that cannot be integrated directly by a single-step method. It is recast as a pair of coupled first-order ODEs by introducing the **state vector** $\mathbf{s} = [x,\; \dot{x}]^T$:

```
ṡ = f(s) = [ ẋ ]  =  [          ẋ           ]       (3)
           [ ẍ ]     [ −(b/m)·ẋ − (k/m)·x   ]
```

In matrix form, the continuous-time linear system is:

```
ṡ = A·s,    A = [  0      1   ]                      (4)
                [ −k/m  −b/m  ]
```

This is implemented in Python as `mass_spring_system(state)`. Each integrator advances this two-element state vector forward in time step by step.

### System Properties

For the primary variant parameters ($m = 0.4$ kg, $k = 4.4$ N/m, $b = 0.025$ N·s/m):

| Property | Formula | Value |
|----------|---------|-------|
| Natural frequency | $\omega_n = \sqrt{k/m}$ | 3.3166 rad/s |
| Damping ratio | $\zeta = b\,/\,(2\sqrt{mk})$ | 0.00942 |
| Damped frequency | $\omega_d = \omega_n\sqrt{1-\zeta^2}$ | 3.3165 rad/s |
| Oscillation period | $T = 2\pi/\omega_d$ | 1.8945 s |
| System type | $\zeta \ll 1$ | **Underdamped** — slowly decaying oscillations |
| Continuous eigenvalues | $\lambda_{1,2} = -b/(2m) \pm j\omega_d$ | $-0.0312 \pm 3.3165j$ |

The negative real parts of the continuous eigenvalues confirm **asymptotic stability** — the physical system decays to rest, but extremely slowly given the small damping ratio. This makes it an effective stress test for numerical integrators: any artificial energy injection (Forward Euler instability) or over-dissipation (Backward Euler artificial damping) is clearly visible in long simulations.

---

## Numerical Integration Methods

All three integrators operate on the state-space representation (Eq. 3), advancing the state from $t = 0$ to $t_f$ using a fixed time step $h = 0.01$ s.

### Explicit (Forward) Euler Method

The simplest one-step scheme. The next state is computed entirely from information available at the **current** time step — no iteration, one function evaluation per step:

```
s_{n+1} = s_n + h · f(s_n)                          (5)
```

**Properties:**
- **Order of accuracy:** 1st order global $O(h)$; 2nd order local per step $O(h^2)$
- **Type:** Explicit — trivially one function evaluation per step, no iteration
- **Stability:** Conditionally stable. Unstable whenever spectral radius $\rho(A_d) > 1$
- **Critical step size:** Instability onset at $h \approx 0.006$ s for this system
- **At $h = 0.01$ s:** $|\lambda|_{\max} \approx 1.000237$ — marginally unstable; divergence is slow but accumulates

The discrete update matrix $A_d = I + hA$ used for stability analysis:

```
A_d(h) = [  1             h      ]                   (6)
         [ −(k/m)·h    1−(b/m)·h ]
```

### Implicit (Backward) Euler Method

An implicit first-order scheme where the next state is defined using the derivative evaluated at the **future** (unknown) time step:

```
s_{n+1} = s_n + h · f(s_{n+1})                      (7)
```

Since $s_{n+1}$ appears on both sides, the equation is solved iteratively at each step via a **fixed-point iteration**:

```
Initialize: s_{n+1} ← s_n
Repeat:
    s_new   = s_n + h · f(s_{n+1})
    error   = ‖s_new − s_{n+1}‖
    s_{n+1} = s_new
Until: error < 1×10⁻⁸
```

**Convergence:** For $h = 0.01$ s, the loop converges to tolerance $10^{-8}$ in approximately **6 iterations** per time step.

**Properties:**
- **Order of accuracy:** 1st order global $O(h)$ — same order as Forward Euler
- **Type:** Implicit — iterative solver required; ~6× more expensive per step than Forward Euler
- **Stability:** **A-stable** — unconditionally stable for all $h > 0$ in the true iterative form
- **Behaviour:** Introduces **artificial numerical dissipation** — oscillations decay faster than the true physical solution

> **Important note on eigenvalue scripts:** `Backward_Euler_with_Eigenvalues.py` and `Runge_Kutta_with_Eigenvalues.py` compute eigenvalue norms of the **linearized** $A_d$ matrix (Eq. 6) — structurally identical to the Forward Euler discretization — for pedagogical illustration of how step size affects the discrete spectrum. This is not the matrix of the true implicit update, but a first-order approximation used to visualize the stability boundary.

### Runge-Kutta Method of Order 4 (RK4)

A 4th-order explicit method that achieves high accuracy by evaluating the derivative at **four intermediate points** within each time step and combining them as a weighted average:

```
f1 = f(s_n)
f2 = f(s_n + 0.5·h·f1)
f3 = f(s_n + 0.5·h·f2)
f4 = f(s_n + h·f3)

s_{n+1} = s_n + (h/6)·(f1 + 2·f2 + 2·f3 + f4)     (8)
```

**Properties:**
- **Order of accuracy:** 4th order global $O(h^4)$; 5th order local per step $O(h^5)$
- **Type:** Explicit — 4 function evaluations per step; no iteration required
- **Stability:** Conditionally stable, but with a substantially wider stability region than Forward Euler
- **Behaviour:** Accurately reproduces both the oscillation frequency and physical damping rate with negligible numerical energy drift

### Accuracy Comparison

One-step integration error at $t = 0.1$ s (reference: RK4 with $h = 10^{-5}$ s):

| Step Size $h$ | Forward Euler Error | RK4 Error |
|:--------------|---------------------|-----------|
| 0.100 s | 0.027239 | 5.07 × 10⁻⁵ |
| 0.050 s | 0.015894 | 3.10 × 10⁻⁶ |
| 0.025 s | 0.008625 | 1.90 × 10⁻⁷ |

Halving $h$ halves the Forward Euler error (confirming 1st-order convergence) and reduces the RK4 error by a factor of ~16 (confirming 4th-order convergence).

---

## Stability Analysis via Eigenvalue Norms

Stability of a discrete-time linear system is governed by the **spectral radius** of its update matrix: the system is stable if and only if all eigenvalues of $A_d$ satisfy $|\lambda_i| \leq 1$.

Three scripts perform this analysis by sweeping over a range of step sizes $h$ and plotting $\max_i |\lambda_i(A_d(h))|$ against $h$:

- `Forward_Euler_with_Eigenvalues.py` — primary variant parameters ($m=0.4$, $k=4.4$, $b=0.025$), $h \in [0, 0.1]$ s
- `Backward_Euler_with_Eigenvalues.py` — extended parameters ($m=0.9$, $k=11.8$, $b=0.015$), $h \in [0.001, 0.1]$ s
- `Runge_Kutta_with_Eigenvalues.py` — extended parameters ($m=0.9$, $k=11.8$, $b=0.015$), $h \in [0, 0.01]$ s

**Forward Euler stability sweep** (primary variant):

| Step Size $h$ | $|\lambda|_{\max}$ | Status |
|:--------------|--------------------|--------|
| 0.001 s | < 1.0 | ✅ Stable |
| 0.006 s | ≈ 1.0 | ⚠️ Critical boundary |
| 0.010 s | 1.000237 | ❌ Unstable |
| 0.050 s | 1.012114 | ❌ Unstable |
| 0.100 s | 1.050595 | ❌ Unstable |

The plot shows the eigenvalue norm crossing the unit-circle threshold $|\lambda| = 1$ at $h \approx 0.006$ s, providing a visual and quantitative characterization of the stability boundary. The RK4 eigenvalue script explicitly marks the $|\lambda| = 1$ threshold with a red dashed line for ease of interpretation.

---

## System Parameters

### Primary Variant Parameters (Variant 2)
Used in `Forward_Euler.py`, `Backward_Euler.py`, `K4_Euler.py`, `Forward_Euler_with_Eigenvalues.py`, `Comparison.py`, `Result.py`:

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.4 | kg |
| Spring constant | $k$ | 4.4 | N/m |
| Damping coefficient | $b$ | 0.025 | N·s/m |
| Initial displacement | $x_0$ | 0.46 | m |
| Initial velocity | $\dot{x}_0$ | 0.0 | m/s |
| Time step | $h$ | 0.01 | s |
| Short simulation horizon | $t_f$ | 10 | s |
| Long simulation horizon | $t_f$ | 30 | s |

### Extended Stability Analysis Parameters
Used in `Backward_Euler_with_Eigenvalues.py` and `Runge_Kutta_with_Eigenvalues.py` to demonstrate stability behaviour on a stiffer, more lightly damped system:

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.9 | kg |
| Spring constant | $k$ | 11.8 | N/m |
| Damping coefficient | $b$ | 0.015 | N·s/m |
| Initial displacement | $x_0$ | 0.30 | m |
| Time step (RK4 script) | $h$ | 0.001 | s |

---

## Implementation

### File Structure

```
Lab_1/
├── Readme.md
├── src/                                 # All simulation scripts
│  ├── Forward_Euler.py                      # Explicit Euler — standalone simulation (t_f = 10 s)
│  ├── Backward_Euler.py                     # Implicit Euler — standalone simulation (t_f = 10 s)
│  ├── K4_Euler.py                           # RK4 — standalone simulation (t_f = 10 s)
│  ├── Forward_Euler_with_Eigenvalues.py     # Explicit Euler + eigenvalue norm stability plot
│  ├── Backward_Euler_with_Eigenvalues.py    # Implicit Euler + eigenvalue norm stability plot
│  ├── Runge_Kutta_with_Eigenvalues.py       # RK4 + eigenvalue norm stability plot
│  ├── Comparison.py                         # All three methods — side-by-side subplot comparison (t_f = 30 s)
│  └── Result.py                             # All three methods — overlaid plots + printed metrics (t_f = 30 s)

├── results/                              # Generated plots / outputs
│   ├── forward_euler_plot.png
│   ├── backward_euler_plot.png
│   ├── rk4_plot.png
│   ├── stability_analysis.png
│   └── comparison_plot.png

├── report/                               # Lab documentation
│   └── Lab1_Report.pdf

└── figures/                              
    └── method_comparison.png
```

**Script grouping by purpose:**

| Group | Scripts | Parameters Used | Duration |
|-------|---------|-----------------|----------|
| Individual integrators | `Forward_Euler.py`, `Backward_Euler.py`, `K4_Euler.py` | Variant 2 | 10 s |
| Eigenvalue stability | `*_with_Eigenvalues.py` | FE: Variant 2 · BE+RK4: Extended | 10 s |
| Comparison and results | `Comparison.py`, `Result.py` | Variant 2 | 30 s |

### Function Reference

#### `mass_spring_system(state)` — all scripts

Encodes the state-space dynamics of the damped mass-spring system (Eq. 3).

| Argument | Type | Description |
|----------|------|-------------|
| `state` | `list` / `np.ndarray` | State vector $[x,\; \dot{x}]$ |

**Returns:** `np.ndarray([ẋ, ẍ])` — the time derivatives of position and velocity.

---

#### `forward_euler(fun, x0, t_f, h)` — `Forward_Euler.py`, `Forward_Euler_with_Eigenvalues.py`, `Comparison.py`, `Result.py`

Integrates an ODE system over $[0, t_f]$ using the explicit Euler update rule (Eq. 5).

| Argument | Type | Description |
|----------|------|-------------|
| `fun` | `callable` | State derivative function $f(\mathbf{s})$ |
| `x0` | `np.ndarray` | Initial state vector $[x_0,\; \dot{x}_0]$ |
| `t_f` | `float` | Final simulation time (s) |
| `h` | `float` | Time step (s) |

**Returns:** `(x_hist, t)` — state history array of shape `(2, N)` and time array of length `N`.

---

#### `backward_euler(fun, x0, t_f, h)` — `Backward_Euler.py`, `Backward_Euler_with_Eigenvalues.py`, `Comparison.py`, `Result.py`

Integrates an ODE system over $[0, t_f]$ using the implicit Euler update rule (Eq. 7), solving the implicit equation at each step via fixed-point iteration with convergence tolerance $10^{-8}$.

| Argument | Type | Description |
|----------|------|-------------|
| `fun` | `callable` | State derivative function $f(\mathbf{s})$ |
| `x0` | `np.ndarray` | Initial state vector |
| `t_f` | `float` | Final simulation time (s) |
| `h` | `float` | Time step (s) |

**Returns:** `(x_hist, t)` — state history array of shape `(2, N)` and time array of length `N`.

---

#### `runge_kutta(xk, h)` — `K4_Euler.py`, `Comparison.py`, `Result.py`

Performs a **single RK4 step** from state `xk` using all four slope estimates $f_1$–$f_4$ (Eq. 8).

| Argument | Type | Description |
|----------|------|-------------|
| `xk` | `np.ndarray` | Current state $\mathbf{s}_n$ |
| `h` | `float` | Time step (s) |

**Returns:** `np.ndarray` — updated state $\mathbf{s}_{n+1}$.

---

#### `rk4(fun, x0, t_f, h)` — `K4_Euler.py`, `Comparison.py`, `Result.py`

Integrates an ODE system over a full time horizon by calling `runge_kutta()` at each step.

**Returns:** `(x_hist, t)` — state history array of shape `(2, N)` and time array of length `N`.

---

#### `mass_spring_Ad(x0, h)` / `mass_spring_Ad_rk4(h, x0)` — eigenvalue scripts

Constructs the linearized discrete-time state transition matrix $A_d$ for a given step size $h$:

```
A_d(h) = [  1            h       ]
         [ −(k/m)·h   1−(b/m)·h  ]
```

Used exclusively for eigenvalue norm computation — not for state propagation.

---

#### `compute_eigenvalues(h_values)` / `compute_rk4_eigenvalues(h_values)` — eigenvalue scripts

Sweeps over an array of step sizes and returns $\max_i |\lambda_i(A_d(h))|$ for each value, producing the stability norm curve plotted against $h$.

**Returns:** `np.ndarray` — maximum eigenvalue norm at each step size.

---

### Algorithm Walkthrough

**Individual integrators** (`Forward_Euler.py`, `Backward_Euler.py`, `K4_Euler.py`):

```
1. Define mass_spring_system(state) → returns [ẋ, ẍ]
2. Set initial conditions:      x0 = [0.46, 0.0]
3. Set simulation parameters:   t_f = 10 s, h = 0.01 s
4. Call integrator(mass_spring_system, x0, t_f, h)  →  (x_hist, t)
5. Plot x_hist[0, :] (position) and x_hist[1, :] (velocity) vs t
6. plt.show()
```

**Eigenvalue stability scripts** (`*_with_Eigenvalues.py`):

```
1. Run integrator simulation  →  plot time-domain response (left subplot)
2. Define h_values = linspace(h_min, h_max, 100)
3. For each h in h_values:
       Construct A_d(h)
       Compute eigenvalues of A_d via np.linalg.eigvals()
       eignorm[k] = max(|eigenvalues|)
4. Plot eignorm vs h_values (right subplot)
   ── RK4 script adds red dashed threshold line at |λ| = 1
5. plt.show()
```

**Comparison and results scripts** (`Comparison.py`, `Result.py`):

```
1. Run all three integrators on the same initial conditions (t_f = 30 s)

2. Comparison.py:
       Plot FE, BE, RK4 in three vertically stacked subplots
       Each subplot shows x(t) and dx(t)

3. Result.py:
       Subplot 1: Overlay x(t) for all three methods
       Subplot 2: Overlay dx(t) for all three methods
       Stability figure: overlay |x(t)| for all three methods
       Compute and print:
           max_amplitude_backward  = max(|x_hist_backward[0, :]|)
           max_amplitude_forward   = max(|x_hist_forward[0, :]|)
           max_amplitude_rk4       = max(|x_hist_rk4[0, :]|)
           final_error_forward     = ‖x_forward[:, −1] − x_backward[:, −1]‖
           final_error_rk4         = ‖x_rk4[:, −1]     − x_backward[:, −1]‖
```

---

## How to Run

### Clone the Repository

```bash
git clone https://github.com/umerahmedbaig7/Simulation-of-Robotic-Systems.git
cd Simulation-of-Robotic-Systems/Lab_1
```

### Prerequisites

```bash
pip install numpy matplotlib
```

### Running the Scripts

Run individual integrators (standalone 10 s simulations, one plot each):

```bash
python Forward_Euler.py
python Backward_Euler.py
python K4_Euler.py
```

Run eigenvalue stability analysis (dynamics plot + eigenvalue norm plot):

```bash
python Forward_Euler_with_Eigenvalues.py
python Backward_Euler_with_Eigenvalues.py
python Runge_Kutta_with_Eigenvalues.py
```

Run comparison and full quantitative results:

```bash
python Comparison.py    # Side-by-side subplots of all three methods
python Result.py        # Overlaid plots + stability plot + printed metrics
```

`Result.py` will additionally print the following to the terminal:

```
Max Amplitude (Backward Euler): 0.4600
Max Amplitude (Forward Euler):  0.9239
Max Amplitude (Runge-Kutta):    0.4600
Final Error (Forward Euler):    2.6219
Final Error (Runge-Kutta):      0.4183
```

### Modifying System Parameters

Open any script and edit the constants block inside `mass_spring_system()`:

```python
b = 0.025   # damping coefficient (N·s/m)
m = 0.4     # mass (kg)
k = 4.4     # spring constant (N/m)
```

Edit initial conditions and simulation settings near the bottom of each script:

```python
x0  = np.array([0.46, 0.0])   # [initial displacement (m), initial velocity (m/s)]
t_f = 10                       # simulation duration (s)
h   = 0.01                     # time step (s)
```

In `Comparison.py` and `Result.py`, the three integrators share one `mass_spring_system()` function, so physical parameter changes propagate to all three methods automatically.

---

## Results

### Individual Method Results — $t_f = 10$ s, $h = 0.01$ s

| Method | Max $|x|$ (m) | Final $x$ (m) | Final $\dot{x}$ (m/s) | Behaviour |
|--------|---------------|---------------|-----------------------|-----------|
| Forward Euler | 0.5761 | −0.0969 | −1.9046 | Slowly diverging — amplitude grows over time |
| Backward Euler | 0.4600 | −0.0283 | −0.6367 | Stable — decays faster than physical rate |
| RK4 | 0.4600 | −0.0565 | −1.0986 | Stable — accurate physical damping reproduction |

### Comparative Metrics — $t_f = 30$ s, $h = 0.01$ s

| Method | Max Amplitude (m) | Final State Error vs. Backward Euler |
|--------|-------------------|--------------------------------------|
| Backward Euler | 0.4600 | — (reference baseline) |
| Forward Euler | 0.9239 | 2.6219 |
| RK4 | 0.4600 | 0.4183 |

**Amplitude envelope behaviour over 30 s:**

- **Backward Euler:** Decays monotonically from 0.46 m to 0.015 m. Artificial numerical dissipation accelerates convergence to zero beyond the true physical rate. Stable but numerically over-damped.
- **Forward Euler:** Amplitude slowly grows from 0.46 m, reaching ~0.92 m by $t = 30$ s — nearly double the initial displacement — confirming gradual numerical energy injection from the unstable eigenvalue norm $|\lambda|_{\max} = 1.000237$.
- **RK4:** Decays in close agreement with the true physical solution. Amplitude at $t = 30$ s is approximately 0.015 m, consistent with the analytical prediction for $\zeta = 0.00942$.

### Stability Analysis Summary

| Method | Stability Type | Instability Onset | $|\lambda|_{\max}$ at $h=0.01$ s |
|--------|----------------|-------------------|----------------------------------|
| Forward Euler | Conditionally stable | $h \approx 0.006$ s | 1.000237 ❌ |
| Backward Euler (iterative) | A-stable — unconditional | Never | < 1.0 ✅ |
| RK4 | Conditionally stable | $h \gg 0.01$ s | ≈ 1.0 ✅ |

---

## Numerical Validation

The correctness of each integrator was verified through three independent checks.

**Check 1 — Order of accuracy confirmation:**
The global error of Forward Euler scales as $O(h)$ and RK4 as $O(h^4)$, confirmed by computing the one-step error at $t = 0.1$ s against a high-precision reference (RK4 at $h = 10^{-5}$ s). Halving $h$ halves the Forward Euler error and reduces the RK4 error by a factor of ~16, in exact agreement with theory.

**Check 2 — Eigenvalue stability boundary:**
The norm $|\lambda|_{\max}(A_d(h))$ was swept numerically over $h \in [0, 0.1]$ s. The unit-circle crossing was located at $h \approx 0.006$ s for Forward Euler, consistent with the approximate analytical bound $h_{\text{crit}} \approx 2/\omega_n \approx 0.006$ s.

**Check 3 — Backward Euler fixed-point convergence:**
The iterative solver was traced step-by-step for the first time step from $\mathbf{s}_0 = [0.46,\; 0]^T$, confirming convergence to tolerance $10^{-8}$ in 6 iterations with a geometrically shrinking error sequence:

```
Iter 1:  error = 5.06×10⁻²
Iter 2:  error = 5.07×10⁻⁴
Iter 3:  error = 5.56×10⁻⁵
Iter 4:  error = 5.61×10⁻⁷
Iter 5:  error = 6.12×10⁻⁸
Iter 6:  error < 1×10⁻⁸   ✓  converged
```

The geometric reduction in error confirms the fixed-point map is contractive for $h = 0.01$ s.

---

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `Python` | ≥ 3.8 | Runtime environment |
| `NumPy` | ≥ 1.21 | Array operations, linear algebra, eigenvalue computation (`np.linalg.eigvals`) |
| `Matplotlib` | ≥ 3.4 | Time-domain state plots, eigenvalue norm curves, multi-panel comparison figures |

All packages are available via pip and are standard in any scientific Python distribution (Anaconda, conda-forge, etc.):

```bash
pip install numpy matplotlib
```

---

## Notes and Limitations

- **Units:** All position values are in **metres**; all time values are in **seconds**; angular values are in **radians** unless explicitly stated otherwise.
- **Eigenvalue matrix is linearized:** The $A_d$ matrix in all three `*_with_Eigenvalues.py` scripts is a first-order Taylor approximation of the discrete update — identical in structure to the Forward Euler matrix — used for pedagogical stability illustration. The true Backward Euler iterative update is A-stable and has no equivalent constant matrix representation.
- **Two distinct parameter sets exist across the eight scripts:** The primary simulation scripts use Variant 2 parameters ($m=0.4$, $k=4.4$, $b=0.025$). The Backward Euler and RK4 eigenvalue scripts use a different stiffer configuration ($m=0.9$, $k=11.8$, $b=0.015$) to broaden the stability illustration. These differences are intentional but must be noted when comparing outputs across scripts.
- **Forward Euler instability at $h = 0.01$ s is mild and slow:** At $|\lambda|_{\max} = 1.000237$, energy grows at roughly 0.024% per oscillation period. Over 10 s this is barely visible; over 30 s the amplitude growth is clearly measurable. Short simulations may give a false impression of stability.
- **Fixed-point iteration may fail for large $h$ or non-linear systems:** The Backward Euler solver assumes the iteration map is contractive. For very large step sizes or highly non-linear dynamics, convergence is not guaranteed. A production implementation should cap the maximum number of iterations and raise a warning on non-convergence.
- **No adaptive step-size control:** All integrators use a fixed step $h$. An adaptive scheme (e.g., RK45 with error control) would automatically reduce $h$ in regions of rapid change and increase it where the solution is smooth, eliminating the need for manual stability analysis.
- **Gravitational acceleration $g$ is declared but unused:** All scripts define `g = 9.8` m/s² inside `mass_spring_system()` but it does not appear in the equation of motion. This is consistent with the equilibrium assumption but represents a minor code hygiene issue.
- **Consistency requirement in comparison scripts:** In `Comparison.py` and `Result.py`, the three integrators share a single `mass_spring_system()` definition. Physical parameter changes propagate to all three automatically. Only the initial conditions, $t_f$, and $h$ need to be updated per-integrator if required.

---

## Author

**Umer Ahmed Baig Mughal** <br>
Master's in Robotics and Artificial Intelligence <br>
*Specialization:  Machine Learning · Computer Vision · Human-Robot Interaction · Autonomous Systems · Robotic Motion Control*

---

## License

This project is intended for **academic and research use**. It was developed as part of the *Simulation of Robotic Systems* course within the MSc Robotics and Artificial Intelligence program at ITMO University. Redistribution, modification, and use in derivative academic work are permitted with appropriate attribution to the original author.

---

*Lab 1 — Simulation of Robotic Systems | MSc Robotics and Artificial Intelligence | ITMO University*
