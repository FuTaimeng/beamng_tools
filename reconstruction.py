import os
import cv2
import json
import torch
import numpy as np
import pypose as pp
import open3d as o3d
import matplotlib.pyplot as plt

suffix = '_leftturn2'
dataroot = f'data{suffix}'
poses = np.loadtxt(os.path.sep.join((dataroot, 'pose.txt')))

grid_length = 0.2
start_frame = 0
end_frame = len(poses)
# end_frame = 2000

config = {
    'grid_length': grid_length,
    'start_frame': start_frame,
    'end_frame': end_frame
}
with open(os.path.sep.join((dataroot, 'cloud_config.txt')), 'w') as f:
    json.dump(config, f)

with open(os.path.sep.join((dataroot, 'cam_param.txt')), 'r') as f:
    cam_param = json.load(f)

resolution = cam_param['resolution']
fov_y = cam_param['fov_y'] /180*np.pi
near_far_planes = cam_param['near_far_planes']
f = resolution[1]/2 / np.tan(fov_y/2)
cx, cy = resolution[0]/2, resolution[1]/2
K = torch.tensor([
    f, 0, cx,
    0, f, cy,
    0,  0,  1
], dtype=torch.float32).view(3,3)
K_inv = torch.linalg.inv(K)
print('K_inv', K_inv)

trans = torch.tensor(cam_param['trans'], dtype=torch.float32)
z = torch.tensor(cam_param['dir'], dtype=torch.float32)
z /= z.norm()
y = -torch.tensor(cam_param['up'], dtype=torch.float32)
y /= y.norm()
x = torch.cross(y, z, dim=0)
x /= x.norm()
rot = torch.stack((x, y, z)).T
q = pp.from_matrix(rot, ltype=pp.SO3_type)
extrinsic = pp.SE3(torch.cat((trans, q.tensor())))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

map = {}

def add_point(key, data):
    if not key in map:
        data.update({'cnt':1})
        map[key] = data
    else:
        grid = map[key]
        cnt = grid['cnt']
        grid['cnt'] = cnt + 1
        for k in data:
            grid[k] = (grid[k]*cnt + data[k]) / (cnt+1)

def gather():
    pts, colors = [], []
    for key, grid in map.items():
        x, y = (key[0]+0.5)*grid_length, (key[1]+0.5)*grid_length
        z = grid['height']
        pts.append([x, y, z])
        colors.append(grid['rgb'])
    return np.array(pts), np.stack(colors)


for idx in range(start_frame, end_frame):
    print('\rprocessing {}/{} ...'.format(idx, end_frame), end='')

    pose = pp.SE3(poses[idx, :7])
    rgb = cv2.imread(os.path.sep.join((dataroot, 'rgb', '{:0>6}.png'.format(idx))), cv2.IMREAD_COLOR)
    seg = cv2.imread(os.path.sep.join((dataroot, 'seg', '{:0>6}.png'.format(idx))), cv2.IMREAD_COLOR)
    depth = np.load(os.path.sep.join((dataroot, 'depth', '{:0>6}.npy'.format(idx))))

    rgb = torch.tensor(rgb, dtype=torch.float32) / 255
    seg = torch.tensor(seg, dtype=torch.float32) / 255
    depth = torch.tensor(depth, dtype=torch.float32)

    mask = torch.zeros(resolution[0]*resolution[1], dtype=bool)
    mask[::10] = True
    mask = torch.logical_and(mask, depth.view(-1) < near_far_planes[1]/10)

    u_lin = torch.linspace(0, resolution[0]-1, resolution[0])
    v_lin = torch.linspace(0, resolution[1]-1, resolution[1])
    u, v = torch.meshgrid(u_lin, v_lin, indexing='xy')
    uv1 = torch.stack([u, v, torch.ones_like(u)]).permute(1,2,0)

    uv1 = uv1.view(-1,3,1)[mask, :, :]
    depth = depth.view(-1,1)[mask, :]
    rgb = rgb.view(-1,3)[mask, :]
    seg = seg.view(-1,3)[mask, :]

    pts = (K_inv.view(1,3,3) @ uv1).squeeze(-1)
    pts *= depth
    # pts = pts / torch.linalg.norm(pts, dim=1).view(-1,1) * depth
    pts = pose @ extrinsic @ pts

    # pts_np = pts.numpy()
    # colors = rgb.numpy()
    # ax.scatter(pts_np[:, 0], pts_np[:, 1], pts_np[:, 2], c=colors)

    for i, pt in enumerate(pts):
        x, y = int(pt[0]//grid_length), int(pt[1]//grid_length)
        add_point((x, y), {'height':float(pt[2]), 'rgb':rgb[i].numpy(), 'seg':seg[i].numpy()})

pts_np, colors = gather()
print('num grids', len(pts_np))
np.save(os.path.sep.join((dataroot, 'cloud.npy')), pts_np)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_np)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.io.write_point_cloud(os.path.sep.join((dataroot, 'cloud.ply')), pcd, write_ascii=False)

ax.scatter(pts_np[:, 0], pts_np[:, 1], pts_np[:, 2], c=colors)
ax.axis('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()