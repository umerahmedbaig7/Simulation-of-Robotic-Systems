import numpy as np
import matplotlib.pyplot as plt

# Mass Spring System
def mass_spring_system(state: list):
    
    # x is position and dx is velocity
    x, dx = state

    # System parameters
    b = 0.025  # damping coefficient in N.s/m
    m = 0.4    # mass in kg
    g = 9.8    # gravitational acceleration (m/s^2)
    k = 4.4    # spring constant in N/m

    # Equation: ddx = -1/m * (b*dx + k*x)
    ddx = -1/m * (b*dx + k*x)

    # Return derivatives as an array
    return np.array([dx, ddx])

# Runge-Kutta 4th Order Method (RK4) function for a single step
def runge_kutta(xk, h):
    # Calculate the four Runge-Kutta increments
    f1 = mass_spring_system(xk)
    f2 = mass_spring_system(xk + 0.5*h*f1)
    f3 = mass_spring_system(xk + 0.5*h*f2)
    f4 = mass_spring_system(xk + h*f3)
    return xk + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)

# Runge-Kutta 4th Order Method for Mass Spring System
def rk4(fun: object, x0: np.ndarray, t_f: float, h: float) -> tuple:
    t = np.arange(0, t_f + h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0
    
    for k in range(len(t) - 1):
        x_hist[:, k + 1] = runge_kutta(x_hist[:, k], h)
    
    return x_hist, t

#Initial Conditions
x_0 = 0.46 # Initial length
dx_0 = 0
x0 = np.array([x_0, dx_0])

# Run the simulation with the Runge-Kutta Method of 4th Order
x_hist3, t_hist3 = rk4(mass_spring_system, x0, 10, 0.01)

# Create a single figure for both plots
plt.figure(figsize=(10, 6))

# Plot position (x) and velocity (dx) over time
plt.plot(t_hist3, x_hist3[0, :], label="$x$")  # Position plot
plt.plot(t_hist3, x_hist3[1, :], label="$dx$") # Velocity plot
plt.title("Runge-Kutta Method of Order 4") # Label title of the used method
plt.xlabel('Time, [sec]')  # Label for the x-axis
plt.ylabel('state')        # Label for the y-axis
plt.legend()               # Show the legend for the plots
plt.grid()                 # Show grid on the plot
plt.show()                 # Display the plot