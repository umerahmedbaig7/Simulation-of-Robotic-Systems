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

# Backward Euler method for Mass Spring System
def backward_euler(fun:object, x0:np.ndarray, t_f:float, h:float) -> tuple:
    # Create a time array from 0 to final time t_f, with step size h
    t = np.arange(0, t_f+h, h)
    x_hist = np.zeros((len(x0), len(t)))
    x_hist[:, 0] = x0 # Set the initial state

    for k in range(len(t)-1):
        e = 1 # an initial error value for interative solution
        x_hist[:, k+1] = x_hist[:, k] 
        while e > 1e-8:
            x_n = x_hist[:, k] + h*fun(x_hist[:, k+1])
            e = np.linalg.norm(x_n - x_hist[:,k+1]) # an error value
            x_hist[:, k+1] = x_n
 
    return x_hist, t

#Initial Conditions
x_0 = 0.46 # Initial length
dx_0 = 0
x0 = np.array([x_0, dx_0])

# Run the simulation with the Backward Euler Method
x_hist3, t_hist3 = backward_euler(mass_spring_system, x0, 10, .01)

# Create a single figure for both plots
plt.figure(figsize=(10, 6))

# Plot position (x) and velocity (dx) over time
plt.plot(t_hist3, x_hist3[0, :], label="$x$")  # Position plot
plt.plot(t_hist3, x_hist3[1, :], label="$dx$") # Velocity plot
plt.title("Implicit Euler Method") # Label title of the used method
plt.xlabel('Time, [sec]')  # Label for the x-axis
plt.ylabel('state')        # Label for the y-axis
plt.ylim(-1.5, 1.5)        # Set y-axis limits from -1.5 to 1.5
plt.legend()               # Show the legend for the plots
plt.grid()                 # Show grid on the plot
plt.show()                 # Display the plot