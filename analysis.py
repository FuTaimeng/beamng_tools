import json
import torch
import numpy as np
import pypose as pp
import matplotlib.pyplot as plt

# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# dataroot = 'data_plane'
# poses = np.loadtxt(f'{dataroot}/pose.txt')
# cam_poses = np.loadtxt(f'{dataroot}/cam_pose.txt')
# cogs = np.loadtxt(f'{dataroot}/cog.txt')
# wheel_speed = np.loadtxt(f'{dataroot}/wheel_speed.txt')

# vehroot = 'vehicle_pickup'
# with open(f'{vehroot}/vehicle_config.txt') as f:
#     config = json.load(f)

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
    
# check_vehicle()


import cv2
# img = cv2.imread('data_straight/map/color/000000.png')
img = cv2.imread('test.png')
# cv2.imshow('img', img)
plt.imshow(img)

# dir = np.array([-1, -1], dtype=float)
# dir /= np.linalg.norm(dir)
# print(dir)
# # rot = -(1 if -dir[1]>=0 else -1) * np.arccos(-dir[0])
# rot = (1 if dir[1]>=0 else -1) * np.arccos(np.dot(dir, np.array([-1, 0], dtype=float)))
# print(np.rad2deg(rot))
# mat = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])

# center = np.array([1400, 700])
# size = 400

# dir = np.array([dir[1], dir[0]])
# tan = np.array([-dir[1], dir[0]])
# plt.scatter([center[0]], [center[1]])
# plt.plot([center[0], center[0]+dir[0]*size/2], [center[1], center[1]+dir[1]*size/2])
# plt.plot([center[0]+(dir[0]+tan[0])*size/2, center[0]+(dir[0]-tan[0])*size/2, center[0]+(-dir[0]-tan[0])*size/2, center[0]+(-dir[0]+tan[0])*size/2, center[0]+(dir[0]+tan[0])*size/2], 
#          [center[1]+(dir[1]+tan[1])*size/2, center[1]+(dir[1]-tan[1])*size/2, center[1]+(-dir[1]-tan[1])*size/2, center[1]+(-dir[1]+tan[1])*size/2, center[1]+(dir[1]+tan[1])*size/2])

# u = np.linspace(-size/2, size/2, size)
# v = np.linspace(-size/2, size/2, size)
# x, y = np.meshgrid(u, v, indexing='xy')
# # x = x.astype(np.float32)
# # y = y.astype(np.float32)
# xy = np.stack([x, y], axis=-1)
# xy = (mat @ xy[..., np.newaxis]).squeeze(-1)
# xy += center
# xy = xy.astype(np.float32)
# print(xy.shape)

# crop = cv2.remap(img, xy, None, interpolation=cv2.INTER_LINEAR)
# cv2.imshow('crop', crop)

# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# # dx = cv2.Sobel(src=gray, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)
# # dy = cv2.Sobel(src=gray, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5)
# dx = cv2.Scharr(gray, cv2.CV_32F, 1, 0)
# dx = cv2.Scharr(gray, cv2.CV_32F, 0, 1)
# cv2.imshow('sobel', dx)
# print(gray.shape, dx.shape, dx.dtype)

idx = np.array(1, dtype=np.float32)
x = cv2.remap(img, idx, idx, cv2.INTER_LINEAR)
print(idx.shape, x.shape)

idx = np.array([1], dtype=np.float32)
x = cv2.remap(img, idx, idx, cv2.INTER_LINEAR)
print(idx.shape, x.shape)

idx = np.array([1, 2], dtype=np.float32)
x = cv2.remap(img, idx, idx, cv2.INTER_LINEAR)
print(idx.shape, x.shape)


idx = np.array([[1, 2]], dtype=np.float32)
x = cv2.remap(img, idx, idx, cv2.INTER_LINEAR)
print(idx.shape, x.shape)

idx = np.array([[1, 2], [1, 2]], dtype=np.float32)
x = cv2.remap(img, idx, idx, cv2.INTER_LINEAR)
print(idx.shape, x.shape)

# plt.show()
# cv2.waitKey(0)