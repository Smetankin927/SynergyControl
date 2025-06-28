# import mujoco
# import numpy as np

# # Load your model
# model = mujoco.MjModel.from_xml_path("2D_simple_pendulum.xml")
# data = mujoco.MjData(model)

# # Set joint positions: 45 degrees (pi/4) each
# data.qpos[:] = [np.pi / 4, np.pi / 4]
# data.qvel[:] = [0.0, 0.0]

# # Forward dynamics
# mujoco.mj_forward(model, data)

# # Create copy with zero velocity to isolate gravity
# data_zero_vel = mujoco.MjData(model)
# data_zero_vel.qpos[:] = data.qpos[:]
# data_zero_vel.qvel[:] = 0
# mujoco.mj_forward(model, data_zero_vel)

# # Gravity torque
# G = data_zero_vel.qfrc_bias.copy()
# print("qpos (rad):", data.qpos)
# print("Gravity torque G(q):", G)


# import mujoco
# import numpy as np

# # Load MuJoCo model
# model = mujoco.MjModel.from_xml_path("2D_simple_pendulum.xml")  # Replace with full path if needed
# data = mujoco.MjData(model)

# # Set joint positions and velocities (customize as needed)
# data.qpos[:] = [np.pi / 4, np.pi / 4]   # 45° for both joints
# data.qvel[:] = [0.5, -0.3]              # some arbitrary velocities

# # Run forward dynamics
# mujoco.mj_forward(model, data)

# # --- 1. Mass matrix M(q) ---
# M = np.zeros((model.nv, model.nv))     # model.nv = number of DoFs
# mujoco.mj_fullM(model, M, data.qM)

# # --- 2. Total bias forces: C(q, q̇)·q̇ + G(q) ---
# bias = data.qfrc_bias.copy()

# # --- 3. Gravity torque G(q): recompute with zero velocity ---
# data_zero_vel = mujoco.MjData(model)
# data_zero_vel.qpos[:] = data.qpos[:]
# data_zero_vel.qvel[:] = 0.0
# mujoco.mj_forward(model, data_zero_vel)
# G = data_zero_vel.qfrc_bias.copy()

# # --- 4. Coriolis and centrifugal: bias - G ---
# C = bias - G

# # --- Optional: Calculate torque for an acceleration (ddq) ---
# ddq = np.array([1.0, -1.0])  # Example joint acceleration
# tau = M @ ddq + C + G

# # --- Output results ---
# np.set_printoptions(precision=4, suppress=True)
# print("Joint positions q (rad):", data.qpos)
# print("Joint velocities q̇ (rad/s):", data.qvel)
# print("\nMass matrix M(q):\n", M)
# print("\nCoriolis & centrifugal C(q, q̇)·q̇:\n", C)
# print("\nGravity torque G(q):\n", G)
# print("\nTotal torque τ = M·ddq + C + G:\n", tau)

import mujoco
import numpy as np

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("2D_simple_pendulum.xml")  # Replace with full path if needed
data = mujoco.MjData(model)

# Set joint positions and velocities
data.qpos[:] = [np.pi / 4, np.pi / 4]   # 45° for both joints
data.qvel[:] = [0.5, -0.3]              # some arbitrary velocities

# --- 1. Mass matrix M(q) ---
# First, run forward dynamics to update internal state
mujoco.mj_forward(model, data)

# Now, extract the mass matrix M(q) (in full format)
M = np.zeros((model.nv, model.nv))     # model.nv = number of DoFs
mujoco.mj_fullM(model, M, data.qM)

# --- 2. Total bias forces: C(q, q̇)·q̇ + G(q) ---
# Extract the bias forces (which include Coriolis and centrifugal terms + gravity)
bias = data.qfrc_bias.copy()

# --- 3. Gravity torque G(q): recompute with zero velocity ---
# Create a new data object to compute gravity forces with zero velocity
data_zero_vel = mujoco.MjData(model)
data_zero_vel.qpos[:] = data.qpos[:]  # Copy current positions
data_zero_vel.qvel[:] = 0.0          # Set velocities to zero
mujoco.mj_forward(model, data_zero_vel)

# Gravity torque G(q)
G = data_zero_vel.qfrc_bias.copy()

# --- 4. Coriolis and centrifugal forces: C(q, q̇)·q̇ ---
# Coriolis and centrifugal forces are the difference between bias forces and gravity
C = bias - G

# --- Optional: Calculate torque for an acceleration (ddq) ---
# Example joint accelerations (arbitrary values)
ddq = np.array([1.0, -1.0])  # Example joint accelerations (rad/s^2)
tau = M @ ddq + C + G

# --- Output results ---
np.set_printoptions(precision=4, suppress=True)
print("Joint positions q (rad):", data.qpos)
print("Joint velocities q̇ (rad/s):", data.qvel)
print("\nMass matrix M(q):\n", M)
print("\nCoriolis & centrifugal C(q, q̇)·q̇:\n", C)
print("\nGravity torque G(q):\n", G)
print("\nTotal torque τ = M·ddq + C + G:\n", tau)
