import numpy as np
import matplotlib.pyplot as plt

def mass_spring_system(state: np.ndarray) -> np.ndarray:
    # Constants
    x, dx = state
    b = 0.015  # Damping coefficient
    m = 0.9    # Mass (kg)
    g = 9.8    # Acceleration due to gravity (m/s²)
    k = 11.8   # Spring constant (N/m)

    # Equations of motion
    ddx = - (b * dx + k * x) / m
    return np.array([dx, ddx])

def runge_kutta(xk: np.ndarray, h: float) -> np.ndarray:
    f1 = mass_spring_system(xk)
    f2 = mass_spring_system(xk + 0.5 * h * f1)
    f3 = mass_spring_system(xk + 0.5 * h * f2)
    f4 = mass_spring_system(xk + h * f3)
    return xk + (h / 6.0) * (f1 + 2 * f2 + 2 * f3 + f4)

def rk4(fun: object, x0: np.ndarray, t_f: float, h: float) -> tuple:
    t = np.arange(0, t_f + h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0
    
    for k in range(len(t) - 1):
        x_hist[:, k + 1] = runge_kutta(x_hist[:, k], h)
    
    return x_hist, t

def mass_spring_Ad_rk4(h: float, x0: np.ndarray) -> np.ndarray:
    # Jacobian for the RK4 method based on the state
    b = 0.015
    m = 0.9
    k = 11.8
    
    # Constructing the Ad matrix
    Ad = np.array([[1, h],
                   [-k/m * h, 1 - (b/m) * h]])
    return Ad

def compute_rk4_eigenvalues(h_values: np.ndarray) -> np.ndarray:
    eignorm = np.zeros(len(h_values))
    for k in range(len(h_values)):
        x0 = np.array([0, 0])  # Use the state at the equilibrium position for stability analysis
        Ad = mass_spring_Ad_rk4(h_values[k], x0)
        eignorm[k] = np.max(np.abs(np.linalg.eigvals(Ad)))
    return eignorm

# Initial conditions and simulation parameters
x0 = np.array([0.3, 0])  # Initial displacement and velocity
t_f = 10  # Final time
h = 0.001  # Time step for integration
h_values = np.linspace(0, 0.01, 100)  # Time steps for stability analysis

# Run the RK4 integration
x_hist, t_hist = rk4(mass_spring_system, x0, t_f, h)

# Stability analysis for RK4 method
eignorm_rk4 = compute_rk4_eigenvalues(h_values)

# Plotting results
fig, axs = plt.subplots(1, 2, figsize=(14, 6))

# Plot for displacement and velocity
axs[0].plot(t_hist, x_hist[0, :], label="$x$ (Displacement)")
axs[0].plot(t_hist, x_hist[1, :], label="$dx$ (Velocity)")
axs[0].set_xlabel('Time [sec]')
axs[0].set_ylabel('State')
axs[0].set_title('Mass-Spring System Dynamics using RK4')
axs[0].legend()
axs[0].grid()

# Plot for eigenvalue norms
axs[1].plot(h_values, eignorm_rk4, label='RK4 Eigenvalue Norm', color='blue')
axs[1].axhline(1, color='red', linestyle='--', label='Stable Threshold (λ=1)')
axs[1].set_xlabel('Time Step (h)')
axs[1].set_ylabel('Max Eigenvalue Norm')
axs[1].set_title('Eigenvalue Norm for Mass-Spring System Stability (RK4)')
axs[1].legend()
axs[1].grid()
axs[1].set_ylim(0, 2)  # Adjust y-limits to emphasize stability

plt.tight_layout()
plt.show()
