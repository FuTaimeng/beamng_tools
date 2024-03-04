import torch
import numpy as np
import pypose as pp
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

poses = np.loadtxt('data/pose.txt')
cam_poses = np.loadtxt('data/cam_pose.txt')
cogs = np.loadtxt('data/cog.txt')

pts = []

# for i in range(50, 60):
#     # print('cog <- pos', cogs[i] - poses[i, :3])
#     # print('cog <- cam_pos', cogs[i] - cam_poses[i, :3])
#     # print('cam_pos <- pos', cam_poses[i, :3] - poses[i, :3])

#     pose = pp.SE3(poses[i])
#     cam_pos = torch.tensor(cam_poses[i, :3], dtype=torch.float32)
#     cam_pos_wrt_vehicle = pose.Inv() @ cam_pos
#     print(cam_pos_wrt_vehicle, cam_pos_wrt_vehicle.norm())
#     pts.append(cam_pos_wrt_vehicle)
#     # input()

for i in range(690, 700):
    # print('cog <- pos', cogs[i] - poses[i, :3])
    # print('cog <- cam_pos', cogs[i] - cam_poses[i, :3])
    # print('cam_pos <- pos', cam_poses[i, :3] - poses[i, :3])

    pose = pp.SE3(poses[i])
    cam_pos = torch.tensor(cam_poses[i, :3], dtype=torch.float32)
    cam_pos_wrt_vehicle = pose.Inv() @ cam_pos
    print(cam_pos_wrt_vehicle, cam_pos_wrt_vehicle.norm())
    pts.append(cam_pos_wrt_vehicle)
    # input()

pts = torch.stack(pts).numpy()
print(np.mean(pts, axis=0))