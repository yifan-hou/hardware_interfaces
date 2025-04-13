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

print("[python] creating manip server")

reset_all_elgato_devices()
server = ms.ManipServer()
if not server.initialize(
    "/home/yifanhou/git/hardware_interfaces_internal/workcell/table_top_manip/config/single_arm_evaluation.yaml"
):
    raise RuntimeError("Failed to initialize server")
server.set_high_level_maintain_position()

print("[python] server created")
while not server.is_ready():
    print("[python] waiting for server to be ready")
    sleep(1)
print("[python] Server is ready")

pose_fb = server.get_pose(1)
print("[python] pose_fb:", pose_fb)

# server.set_high_level_maintain_position()
# bgr = np.zeros((1080, 1080, 3), dtype=np.uint8)
# cv2.namedWindow("image")

server.set_force_controlled_axis(np.eye(6), 6)
print("[python] done set_force_controlled_axis")

pose_cmd = copy.deepcopy(pose_fb)
pose_cmds = np.zeros((7, 5))

log_pose_x = []
log_pose_y = []
log_pose_z = []

log_pose_cmd_x = []
log_pose_cmd_y = []
log_pose_cmd_z = []


input("Press Enter to start the test...")

deltas = np.array([0.00, -0.00])
for i in range(2):
    wrench = server.get_robot_wrench(1)
    pose_fb = server.get_pose(1)

    # rgb_row_combined = server.get_camera_rgb()
    # # rgb = rgb_row_combined.reshape((1080, 1080, 3))
    # # # rgb to bgr
    # # bgr = rgb[:, :, ::-1]
    # bgr[:, :, 2] = rgb_row_combined[0:1080, 0:1080]
    # bgr[:, :, 1] = rgb_row_combined[1080:2160, 0:1080]
    # bgr[:, :, 0] = rgb_row_combined[2160:3240, 0:1080]
    # # print(f"image size: {rgb_row_combined.shape}, rgb size: {rgb.shape}")

    # cv2.imshow("image", bgr)
    # key = cv2.waitKey(20)

    for j in range(5):
        pose_cmds[:, j] = pose_fb.reshape((7,))
        pose_cmds[0, j] += deltas[i] * j
    timepoints_ms = (
        np.array([0.0, 400, 800, 1200, 1600]) + server.get_timestamp_now_ms()
    )
    server.schedule_waypoints(pose_cmds, timepoints_ms)
    pose_fb = server.get_pose(1)
    log_pose_x.append(pose_fb[0])
    log_pose_y.append(pose_fb[1])
    log_pose_z.append(pose_fb[2])

    log_pose_cmd_x.append(pose_cmd[0])
    log_pose_cmd_y.append(pose_cmd[1])
    log_pose_cmd_z.append(pose_cmd[2])

    sleep(10)

sleep(0.5)
server.join_threads()
print("start drawing")

# fig, axs = plt.subplots(1, 3, figsize=(9, 3), sharey=True)

# # plot log_pose_x, log_pose_y, log_pose_z
# axs[0].plot(log_pose_x, label="pose_x")
# axs[1].plot(log_pose_y, label="pose_y")
# axs[2].plot(log_pose_z, label="pose_z")

# axs[0].plot(log_pose_cmd_x, label="pose_cmd_x")
# axs[1].plot(log_pose_cmd_y, label="pose_cmd_y")
# axs[2].plot(log_pose_cmd_z, label="pose_cmd_z")

# axs[0].legend()
# axs[1].legend()
# axs[2].legend()

# plt.show()
