import os
import sys

SCRIPT_PATH = os.path.abspath(os.path.dirname(__file__))
PACKAGE_PATH = os.path.join(SCRIPT_PATH, "../../")
sys.path.append(os.path.join(SCRIPT_PATH, "../../../"))

import manip_server_pybind as ms
import numpy as np
from time import sleep
import cv2
import copy
import matplotlib.pyplot as plt
from einops import rearrange

from PyriteUtility.umi_utils.usb_util import reset_all_elgato_devices
from PyriteUtility.spatial_math import spatial_utilities as su

print("[python] creating manip server")

reset_all_elgato_devices()
server = ms.ManipServer()
if not server.initialize(
    "/home/chendong/workspace/hardware_interfaces/workcell/table_top_manip/config/single_arm_evaluation.yaml"
):
    raise RuntimeError("Failed to initialize server")
server.set_high_level_maintain_position()

print("[python] server created")
while not server.is_ready():
    print("[python] waiting for server to be ready")
    sleep(1)
print("[python] Server is ready")

# calibrate
server.calibrate_robot_wrench(100)
pose_fb = server.get_pose(1)
print("[python] pose_fb:", pose_fb)

server.set_force_controlled_axis(np.eye(6), 6)
print("[python] done set_force_controlled_axis")

pose_cmd = copy.deepcopy(pose_fb)

N = 120
magnitude = 0.05
dt_ms = 30  # milliseconds
pose_cmds = np.zeros((N, 7))
t = np.linspace(0, 2 * np.pi, N)
step = np.concatenate([np.zeros(N//3), np.ones(N//3), np.zeros(N//3)])  # 0 for first half, 1 for second half


SE3_fb = su.pose7_to_SE3(pose_fb.reshape((1, 7))).reshape((4,4))
q_fb = pose_fb[3:].reshape((1, 4))
theta = 5. * 3.1416 / 180.
n = np.array([0, 1, 0])
Ax = su.wedge3(n)
SO3_delta = np.eye(3) + np.sin(theta) * Ax + (1 - np.cos(theta)) * Ax * Ax
SE3_delta = np.eye(4)
SE3_delta[:3, :3] = SO3_delta
SE3_cmd = SE3_delta * SE3_fb
q_cmd = su.SO3_to_quat(SE3_cmd[:3, :3]).reshape((1,4))
Ns = np.ones((N//3, 1))
q_all = np.concatenate([Ns @ q_fb, Ns @ q_cmd, Ns @ q_fb], axis=0)


# pose_cmds[:, 0] = pose_fb[0] + magnitude * np.sin(t)  # x position
# pose_cmds[:, 0] = pose_fb[0] + magnitude * step  # x position
pose_cmds[:, 0] = pose_fb[0] + magnitude * np.sin(t)  # x position
pose_cmds[:, 1] = pose_fb[1] + magnitude * np.sin(t)  # y position
pose_cmds[:, 2] = pose_fb[2] + magnitude * np.sin(t)  # z position
pose_cmds[:, 3] = pose_fb[3]  # quaternion x
pose_cmds[:, 4] = pose_fb[4]  # quaternion y
pose_cmds[:, 5] = pose_fb[5]  # quaternion z
pose_cmds[:, 6] = pose_fb[6]  # quaternion w
# pose_cmds[:, 3:] = q_all

log_pose_x = []
log_pose_y = []
log_pose_z = []
log_time = []

input("Press Enter to start the test...")
deltas = np.array([0.00, -0.00])

timepoints_cmd_ms = (
    server.get_timestamp_now_ms() + np.arange(N) * dt_ms
)
server.schedule_waypoints(pose_cmds.T, timepoints_cmd_ms)

for i in range(N):
    pose_fb = server.get_pose(1)
    log_pose_x.append(pose_fb[0])
    log_pose_y.append(pose_fb[1])
    log_pose_z.append(pose_fb[2])
    log_time.append(server.get_timestamp_now_ms())

    sleep(dt_ms / 1000.0)  # Convert milliseconds to seconds


server.join_threads()
print("start drawing")

x_error = np.abs(log_pose_x[-1] - pose_cmds[-1, 0])
z_error = np.abs(log_pose_z[-1] - pose_cmds[-1, 2])
print("X_error: ", x_error, ", z_error: ", z_error)

fig, axs = plt.subplots(1, 3, figsize=(12, 5), sharey=False)

# plot log_pose_x, log_pose_y, log_pose_z
axs[0].plot(log_time, log_pose_x, label="pose_x")
axs[1].plot(log_time, log_pose_y, label="pose_y")
axs[2].plot(log_time, log_pose_z, label="pose_z")

axs[0].plot(timepoints_cmd_ms, pose_cmds[:, 0], label="pose_cmd_x")
axs[1].plot(timepoints_cmd_ms, pose_cmds[:, 1], label="pose_cmd_y")
axs[2].plot(timepoints_cmd_ms, pose_cmds[:, 2], label="pose_cmd_z")

axs[0].legend()
axs[1].legend()
axs[2].legend()

# enable grid
axs[0].grid()
axs[1].grid()
axs[2].grid()

plt.show()