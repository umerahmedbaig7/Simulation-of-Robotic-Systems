import numpy as np
import matplotlib.pyplot as plt

# Define the mass-spring system
def mass_spring_system(state: list) -> np.ndarray:
    x, dx = state
    b = 0.025  # Damping coefficient
    m = 0.4    # Mass (kg)
    g = 9.8    # Acceleration due to gravity (m/s^2)
    k = 4.4    # Spring constant
    ddx = -1/m * (b*dx + k*x)
    return np.array([dx, ddx])

# Backward Euler Integration Function
def backward_euler(fun: object, x0: np.ndarray, t_f: float, h: float) -> tuple:
    t = np.arange(0, t_f + h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0

    for k in range(len(t) - 1):
        e = 1  # initial error
        x_hist[:, k + 1] = x_hist[:, k]  # Start with the previous value
        while e > 1e-8:
            x_n = x_hist[:, k] + h * fun(x_hist[:, k + 1])
            e = np.linalg.norm(x_n - x_hist[:, k + 1])  # Calculate error
            x_hist[:, k + 1] = x_n
 
    return x_hist, t

# Forward Euler Integration Function
def forward_euler(fun: object, x0: np.ndarray, t_f: float, h: float) -> tuple:
    t = np.arange(0, t_f + h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0

    for k in range(len(t) - 1):
        x_hist[:, k + 1] = x_hist[:, k] + h * fun(x_hist[:, k])
 
    return x_hist, t

# Runge-Kutta Integration Function
def runge_kutta(xk, h):
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

# Initial conditions
x0 = np.array([0.46, 0])
t_f = 30
h = 0.01

# Run all three integration methods
x_hist_backward, t_hist_backward = backward_euler(mass_spring_system, x0, t_f, h)
x_hist_forward, t_hist_forward = forward_euler(mass_spring_system, x0, t_f, h)
x_hist_rk4, t_hist_rk4 = rk4(mass_spring_system, x0, t_f, h)

# Plotting results
plt.figure(figsize=(4, 8))

# Forward Euler
plt.subplot(3, 1, 1)
plt.plot(t_hist_forward, x_hist_forward[0, :], label="$x$")
plt.plot(t_hist_forward, x_hist_forward[1, :], label="$dx$")
plt.title("Forward Euler Integration")
plt.xlabel('Time [sec]')
plt.ylabel('State')
plt.legend()
plt.grid()

# Backward Euler
plt.subplot(3, 1, 2)
plt.plot(t_hist_backward, x_hist_backward[0, :], label="$x$")
plt.plot(t_hist_backward, x_hist_backward[1, :], label="$dx$")
plt.title("Backward Euler Integration")
plt.xlabel('Time [sec]')
plt.ylabel('State')
plt.legend()
plt.grid()

# Runge-Kutta
plt.subplot(3, 1, 3)
plt.plot(t_hist_rk4, x_hist_rk4[0, :], label="$x$")
plt.plot(t_hist_rk4, x_hist_rk4[1, :], label="$dx$")
plt.title("Runge-Kutta of 4th Order Integration")
plt.xlabel('Time [sec]')
plt.ylabel('State')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
