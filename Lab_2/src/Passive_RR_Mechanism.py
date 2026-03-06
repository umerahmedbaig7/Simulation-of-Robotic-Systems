import time
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt  # Corrected import statement

# Load MuJoCo model and data
m = mujoco.MjModel.from_xml_path('D:\ITMO University\Semester 1\Simulation of Robotic Systems\Lab Task 2\Task2.xml')
d = mujoco.MjData(m)

# Initialize lists for joint values
x_values = []
y_values = []

# Launch passive viewer
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()

    # Run the simulation for 5 seconds
    while viewer.is_running() and time.time() - start < 15:
        step_start = time.time()

        # Append joint positions to the respective lists
        y_values.append(float(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'one')]]))
        x_values.append(float(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'two')]]))

        # Step simulation
        mujoco.mj_step(m, d)

        # Update viewer settings
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Sync viewer
        viewer.sync()

        # Ensure correct simulation timing
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# Plot the first joint's position
plt.figure()
plt.plot(y_values, label="J1")
plt.ylabel("J1")
plt.title("Joint One Position Over Time")
plt.grid(True)
plt.legend()
plt.show()

# Plot the second joint's position
plt.figure()
plt.plot(x_values, label="J2")
plt.ylabel("J2")
plt.title("Joint Two Position Over Time")
plt.grid(True)
plt.legend()
plt.show()

# Plot the positions of both joints on the same graph
plt.figure()
plt.plot(y_values, label="Joint One (J1)", color="blue")
plt.plot(x_values, label="Joint Two (J2)", color="red")
plt.xlabel("Simulation Steps")
plt.ylabel("Joint Position")
plt.title("Joint Positions Over Time")
plt.legend()
plt.grid(True)
plt.show()
