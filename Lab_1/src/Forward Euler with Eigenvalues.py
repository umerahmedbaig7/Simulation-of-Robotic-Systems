import numpy as np
import matplotlib.pyplot as plt

def mass_spring_system(state: np.ndarray) -> np.ndarray:
    # Constants
    x, dx = state
    b = 0.025  # Damping coefficient
    m = 0.4    # Mass (kg)
    g = 9.8    # Acceleration due to gravity (m/s²)
    k = 4.4    # Spring constant (N/m)

    # Equations of motion
    ddx = - (b * dx + k * x) / m
    return np.array([dx, ddx])

def forward_euler(fun, x0: np.ndarray, t_f: float, h: float) -> tuple:
    t = np.arange(0, t_f + h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0

    for k in range(len(t) - 1):
        x_hist[:, k + 1] = x_hist[:, k] + h * fun(x_hist[:, k])
    
    return x_hist, t

def mass_spring_Ad(x0: np.ndarray, h: float) -> np.ndarray:
    # Discrete-time state transition matrix for the mass-spring system
    b = 0.025
    m = 0.4
    k = 4.4
    Ad = np.array([[1, h],
                   [-k/m * h, 1 - b/m * h]])
    return Ad

def compute_eigenvalues(h_values: np.ndarray) -> np.ndarray:
    eignorm = np.zeros(len(h_values))
    for k in range(len(h_values)):
        Ad = mass_spring_Ad(np.array([0, 0]), h_values[k])
        eignorm[k] = np.max(np.abs(np.linalg.eigvals(Ad)))
    return eignorm

# Initial conditions and simulation parameters
x0 = np.array([0.46, 0])  # Initial displacement and velocity
t_f = 10  # Final time
h = 0.01  # Time step

# Run the forward Euler integration
x_hist3, t_hist3 = forward_euler(mass_spring_system, x0, t_f, h)

# Plotting results for the mass-spring system
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(t_hist3, x_hist3[0, :], label="$x$ (Displacement)")
plt.plot(t_hist3, x_hist3[1, :], label="$dx$ (Velocity)")
plt.xlabel('Time [sec]')
plt.ylabel('State')
plt.title('Mass-Spring System Dynamics')
plt.legend()
plt.grid()

# Stability analysis
h_values = np.linspace(0, 0.1, 100)
eignorm = compute_eigenvalues(h_values)

# Plotting eigenvalues norm
plt.subplot(1, 2, 2)
plt.plot(h_values, eignorm)
plt.xlabel('Time Step (h)')
plt.ylabel('Max Eigenvalue Norm')
plt.title('Eigenvalue Norm for Mass-Spring System')
plt.grid()
plt.tight_layout()
plt.show()
