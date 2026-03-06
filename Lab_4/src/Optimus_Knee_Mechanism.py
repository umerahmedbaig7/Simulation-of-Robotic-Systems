import numpy as np
import time
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Path to your XML model file
MODEL_NAME = 'd:\ITMO University\Semester 1\Simulation of Robotic Systems\Lab Task 4\MuJoCo_Optimus_Knee_Model.xml'

# Constants for simulation
MAX_CTRL = 0.01  # Maximum actuator control range
CTRL_INCREMENT = 0.0005  # Step increment for actuator control
SIMULATION_TIME = 10  # Total simulation duration in seconds

# Function to get Euler angles from a rotation matrix
def get_euler_angles_from_rotation_matrix(rotation_matrix):
    rotations = R.from_matrix(rotation_matrix)
    euler = rotations.as_euler('xyz', degrees=True)
    return euler

# Function to calculate the rotation of a given geometry
def get_l1_rotation(model, data, geom_name):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    rotation_matrix = np.reshape(data.geom_xmat[geom_id], (3, 3))
    rotation = R.from_matrix(rotation_matrix)
    euler_angles = rotation.as_euler('xyz', degrees=True)
    return euler_angles

# Function to control the actuator
def controller(model, data, current_ctrl):
    if current_ctrl < model.actuator_ctrlrange[0][1]:
        current_ctrl += CTRL_INCREMENT
    else:
        current_ctrl = model.actuator_ctrlrange[0][1]
    data.ctrl[0] = current_ctrl
    return current_ctrl

# Main simulation function
def simulate():
    # Load the model and initialize data
    model = mujoco.MjModel.from_xml_path(MODEL_NAME)
    data = mujoco.MjData(model)

    # Initialize actuator and angle tracking
    actuator_positions = []
    angle_deviations = []

    # Initialize initial states
    initial_actuator_position = data.actuator_length[0]
    initial_euler_angles = get_l1_rotation(model, data, 'Length_1')
    initial_angle_y = initial_euler_angles[1]

    # Debug: Print initial state
    print("Initial actuator position:", initial_actuator_position)
    print("Initial Y angle:", initial_angle_y)

    # Open the viewer for rendering
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        current_ctrl = model.actuator_ctrlrange[0][0]  # Start at the minimum control value

        # Run the simulation loop
        while viewer.is_running() and time.time() - start_time < SIMULATION_TIME:
            step_start = time.time()

            # Calculate the current rotation and angle deviation
            euler_angles = get_l1_rotation(model, data, 'Length_1')
            current_angle_y = euler_angles[1] - initial_angle_y  # Deviation from initial angle
            angle_deviations.append(current_angle_y)

            # Calculate actuator position deviation
            current_actuator_position = data.actuator_length[0]
            actuator_deviation = current_actuator_position - initial_actuator_position
            actuator_positions.append(actuator_deviation)

            # Apply controller logic
            current_ctrl = controller(model, data, current_ctrl)

            # Step the simulation and render
            mujoco.mj_step(model, data)
            viewer.sync()

            # Sleep to match real-time simulation step
            next_step = model.opt.timestep - (time.time() - step_start)
            if next_step > 0:
                time.sleep(next_step)

    # Plot the results
    plt.figure()
    plt.plot(actuator_positions, angle_deviations, label="Angle deviation vs Actuator position", color="purple")
    plt.xlabel("Actuator Deviation")
    plt.ylabel("Y Angle Deviation (degrees)")
    plt.title("Length_1 Angle Deviation vs Actuator Position")
    plt.grid(True)
    plt.legend()
    plt.show()

# Entry point
if __name__ == '__main__':
    simulate()

