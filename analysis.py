import json
import torch
import numpy as np
import pypose as pp
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

dataroot = 'data_plane'
poses = np.loadtxt(f'{dataroot}/pose.txt')
cam_poses = np.loadtxt(f'{dataroot}/cam_pose.txt')
cogs = np.loadtxt(f'{dataroot}/cog.txt')
wheel_speed = np.loadtxt(f'{dataroot}/wheel_speed.txt')

vehroot = 'vehicle_pickup'
with open(f'{vehroot}/vehicle_config.txt') as f:
    config = json.load(f)

def extrinsic_trans(st=0, end=10):
    pts = []
    for i in range(st, end):
        pose = pp.SE3(poses[i, :7])
        cam_pos = torch.tensor(cam_poses[i, :3], dtype=torch.float32)
        cam_pos_wrt_vehicle = pose.Inv() @ cam_pos
        # print(cam_pos_wrt_vehicle, cam_pos_wrt_vehicle.norm())
        pts.append(cam_pos_wrt_vehicle)
    pts = torch.stack(pts)
    print('mean:', torch.mean(pts, dim=0))
    return pts

def check_velocity(st=1, end=10):
    for i in range(st, end):
        p1 = np.array(poses[i-1, :3])
        p2 = np.array(poses[i+1, :3])
        v = np.array(poses[i, 7:])
        v_calc = (p2 - p1) / 0.2
        ws = wheel_speed[i]
        print(v, v_calc)
        print(np.linalg.norm(v), ws)

def check_vehicle():
    mass = np.array(config['mass'])
    mass_pts = np.array(config['mass_pts'])
    M = np.sum(mass)
    print('M', M)
    cog_calc = np.sum(mass.reshape(-1, 1) * mass_pts, axis=0) / M
    print('COG', cog_calc)

# extrinsic_trans(0, 50)
# extrinsic_trans(1350, 1355)
# extrinsic_trans(0, 1355)

# check_velocity(100, 120)
    
check_vehicle()