import time
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

# Load the MuJoCo model and data
m = mujoco.MjModel.from_xml_path('d:\ITMO University\Semester 1\Simulation of Robotic Systems\Lab Task 3\MuJoCo_Model_with_Actuators.xml')
d = mujoco.MjData(m)

# Target position for joint B
amplitude = 1.073  # radians
frequency = 3.2  # Hz
constant_angle = 1.564  # radians

# Initialize data for plotting
joint_a_positions = []
joint_b_positions = []
joint_a_velocities = []
joint_b_velocities = []
end_effector_positions = {"x": [], "y": [], "z": []}
end_effector_orientations_pitch = []
target_positions = []  # To store the sinusoidal target positions

# Function to compute the rotation matrix and Euler angles (pitch, roll, yaw)
def get_rotation(model, data, site_name):
    wall_orientation = data.geom_xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'right_wall')]
    rotation_mat = np.reshape(wall_orientation, (3, 3))
    rotation = R.from_matrix(rotation_mat)
    euler_angles = rotation.as_euler('xyz', degrees=True) 
    return euler_angles

# PD Controller function
def controller(model, data, joint_name, target_position):

    KP = 0.005  # Proportional gain
    KD = 0.005  # Derivative gain

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    joint_pos_addr = model.jnt_qposadr[joint_id]
    joint_vel_addr = model.jnt_dofadr[joint_id]

    current_position = data.qpos[joint_pos_addr]
    current_velocity = data.qvel[joint_vel_addr]
    position_error = target_position - current_position
    control_signal = KP * position_error - KD * current_velocity

    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
    data.ctrl[actuator_id] = control_signal  

# Function to get the end-effector position and orientation
def get_end_effector_data():
    site_name = "right_wall"  # Name of the end-effector site
    site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'right_wall')
    end_effector_pos = d.geom_xpos[site_id]
    end_effector_positions["x"].append(end_effector_pos[0])
    end_effector_positions["y"].append(end_effector_pos[1])
    end_effector_positions["z"].append(end_effector_pos[2])
    euler_angles = get_rotation(m, d, site_name)
    end_effector_orientations_pitch.append(euler_angles[1])  # Pitch is the second element

# Launch viewer with simulation
with mujoco.viewer.launch_passive(m, d) as viewer:
    start_time = time.time()

    # Set the timestep to slow down the simulation
    timestep = 0.002  # 100 Hz (you can adjust this to make it slower)
    m.opt.timestep = timestep

    # Run the simulation for 10 seconds
    while viewer.is_running() and time.time() - start_time < 10:
        sim_time = time.time() - start_time
        target_position_joint_b = amplitude * np.sin(constant_angle * sim_time)
        target_positions.append(target_position_joint_b)
        controller(m, d, "B", target_position_joint_b)

        # Record joint positions
        joint_a_positions.append(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'A')]])
        joint_b_positions.append(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'B')]])

        # Record joint velocities
        joint_a_velocities.append(d.qvel[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'A')]])
        joint_b_velocities.append(d.qvel[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'B')]])

        # Record end-effector data
        get_end_effector_data()

        # Step the simulation
        mujoco.mj_step(m, d)
        
        # Synchronize the viewer
        viewer.sync()

        time.sleep(timestep)

# Plot joint positions over time
plt.figure()
plt.plot(joint_a_positions, label="Joint A Position")
plt.plot(joint_b_positions, label="Joint B Position")
plt.xlabel("Time Steps")
plt.ylabel("Position (rad)")
plt.title("Joint Positions A & B Over Time")
plt.grid(True)
plt.legend()
plt.show()

# Plot joint velocities over time
plt.figure()
plt.plot(joint_a_velocities, label="Joint A Velocity")
plt.plot(joint_b_velocities, label="Joint B Velocity")
plt.xlabel("Time Steps")
plt.ylabel("Velocity (rad/s)")
plt.title("Joint Velocities A & B Over Time")
plt.grid(True)
plt.legend()
plt.show()

# Plot end-effector position
plt.figure()
plt.plot(end_effector_positions["x"], label="X Position")
plt.plot(end_effector_positions["y"], label="Y Position")
plt.plot(end_effector_positions["z"], label="Z Position")
plt.xlabel("Time Steps")
plt.ylabel("Position (m)")
plt.title("End-Effector Position (right_wall) Over Time")
plt.grid(True)
plt.legend()
plt.show()

# Plot end-effector orientation (pitch)
plt.figure()
plt.plot(end_effector_orientations_pitch, label="Pitch")
plt.xlabel("Time Steps")
plt.ylabel("Pitch (degrees)")
plt.title("End-Effector Pitch Orientation Over Time (right_wall)")
plt.grid(True)
plt.legend()
plt.show()
