import mujoco
import numpy as np

# Load model and data
model = mujoco.MjModel.from_xml_path("2D_simple_pendulum.xml")  # replace with your model path
data = mujoco.MjData(model)

# Set joint states (optional - set a specific configuration)
data.qpos[:] = [0.1]        # for example, small angle
data.qvel[:] = [0.0]        # zero velocity

# Run forward dynamics to populate internal values
mujoco.mj_forward(model, data)

# 1. Mass matrix M(q)
M = np.zeros((model.nv, model.nv))
mujoco.mj_fullM(model, M, data.qM)  # fills M from internal sparse format

# 2. Bias forces: C(q, q̇)·q̇ + G(q)
bias = data.qfrc_bias.copy()

# 3. Gravity G(q) alone (set velocity to 0)
data_zero_vel = mujoco.MjData(model)
data_zero_vel.qpos[:] = data.qpos[:]
data_zero_vel.qvel[:] = 0
mujoco.mj_forward(model, data_zero_vel)
G = data_zero_vel.qfrc_bias.copy()

# 4. Coriolis/centrifugal C(q, q̇)·q̇ = bias - G
C = bias - G

# Print results
print("Mass matrix M(q):\n", M)
print("Coriolis and centrifugal C(q, q̇)·q̇:\n", C)
print("Gravity G(q):\n", G)
