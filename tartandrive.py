import os
import cv2
import time
import json
import torch
import numpy as np
import pypose as pp
import open3d as o3d
import matplotlib.pyplot as plt


root = 'drive'
name = '20210826_34'
map_folder = 'map2'

data = torch.load(f'{root}/{name}.pt')

st = 20
end = len(data['observation']['state'])
N = end - st

pose = data['observation']['state'][st:end, 0:7]
pose = pp.SE3(pose)

rgbmap = data['observation']['rgbmap'][st:end]
rgbmap = rgbmap.permute(0, 2, 3, 1)

heightmap = data['observation']['heightmap'][st:end]
heightmap = heightmap.squeeze(1)

image = data['observation']['image_rgb'][st:end]
image = image.permute(0, 2, 3, 1)

# mask = torch.logical_and(torch.sum(rgbmap, dim=-1) > 0.1, heightmap < 3)

y = torch.linspace(0, 10, 64)
x = torch.linspace(5, -5, 64)
yy, xx = torch.meshgrid([y, x], indexing='ij')
xx = xx.unsqueeze(0).expand(N, -1, -1)
yy = yy.unsqueeze(0).expand(N, -1, -1)

l_points = torch.stack([xx, yy, heightmap], dim=-1)

rot = pose.rotation().Log().tensor()
rot[..., [0,1]] = 0
rot = pp.so3(rot).Exp().tensor()
trans = pose.translation()
trans[..., 2] = 0
pose_map = torch.cat([trans, rot], dim=-1)
pose_map = pp.SE3(pose_map)

w_points = pose_map.unsqueeze(1).unsqueeze(1) @ l_points

# c = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1)]
# points, colors = [], []
# for i in range(5):
#     pts = w_points[30+i*5][mask[30+i*5]].numpy().reshape(-1, 3)
#     # pts = w_points[50+i*5].numpy().reshape(-1, 3)
#     col = np.zeros_like(pts)
#     col[:] = c[i]
#     points.append(pts)
#     colors.append(col)
# points = np.concatenate(points, axis=0)
# colors = np.concatenate(colors, axis=0)
# # points = w_points[mask].numpy()
# # colors = rgbmap[mask].numpy()
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)
# pcd.colors = o3d.utility.Vector3dVector(colors)
# o3d.io.write_point_cloud('drive/cloud.ply', pcd, write_ascii=False)

from reconstruction import Map

depth_bit = 16

grid_length = 0.02 * 5
block_size = int(1000 / 5)

terrian_map = Map(grid_length, block_size, 'minimum', depth_bit)
for i in range(N):
    points = w_points[i].numpy().reshape(-1, 3)
    colors = rgbmap[i].numpy().reshape(-1, 3)
    annotations = np.zeros(len(colors), dtype=int)
    terrian_map.add_points(points, colors, annotations)

t0 = time.time()
heights, colors, annotations, infos = terrian_map.get_images()
t1 = time.time()
print('get_images time:', t1-t0)

if os.path.isdir(f'{root}/{name}/{map_folder}'):
    os.system('rm -r '+f'{root}/{name}/{map_folder}')
os.makedirs(f'{root}/{name}/{map_folder}/height', exist_ok=True)
os.makedirs(f'{root}/{name}/{map_folder}/color', exist_ok=True)
os.makedirs(f'{root}/{name}/{map_folder}/annotation', exist_ok=True)

infos.update({
    'num_block':len(heights), 
    'grid_length':grid_length / 5, 
    'block_size':block_size * 5, 
    'depth_bit':depth_bit,
    'start_frame':st,
    'end_frame':end,
})
with open(f'{root}/{name}/{map_folder}/info.txt', 'w') as f:
    json.dump(infos, f)

for i in range(len(heights)):
    img = cv2.resize(heights[i], None, fx=5, fy=5)
    cv2.imwrite(f'{root}/{name}/{map_folder}/height/{i:0>6d}.png', img)
    img = cv2.resize(colors[i], None, fx=5, fy=5)
    cv2.imwrite(f'{root}/{name}/{map_folder}/color/{i:0>6d}.png', img)
    img = cv2.resize(annotations[i], None, fx=5, fy=5)
    cv2.imwrite(f'{root}/{name}/{map_folder}/annotation/{i:0>6d}.png', img)

points, colors = terrian_map.get_pointcloud()
points, colors = points[::100], colors[::100]
points_colors = np.concatenate((points, colors), axis=-1)
# np.save(os.path.sep.join((dataroot, 'cloud.npy')), points_colors)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.io.write_point_cloud(f'{root}/{name}/{map_folder}/cloud.ply', pcd, write_ascii=False)

os.makedirs(f'{root}/{name}/heightmap', exist_ok=True)
os.makedirs(f'{root}/{name}/rgbmap', exist_ok=True)
os.makedirs(f'{root}/{name}/rgb', exist_ok=True)
for i in range(N):
    img = (heightmap[i].numpy()/5*255).astype(np.uint8)
    cv2.imwrite(f'{root}/{name}/heightmap/{i:0>6d}.png', img)
    img = (rgbmap[i].numpy()*255).astype(np.uint8)
    cv2.imwrite(f'{root}/{name}/rgbmap/{i:0>6d}.png', img)
    img = (image[i].numpy()*255).astype(np.uint8)
    cv2.imwrite(f'{root}/{name}/rgb/{i:0>6d}.png', img)

fig = plt.figure('traj')
ax = fig.add_subplot(projection='3d')
pos = pose.translation().numpy()
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2])
for i in range(0, N, 10):
    pt = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=torch.float32)
    pt = pose[i] @ pt
    for j in range(3):
        plt.plot([pos[i, 0], pt[j, 0]], [pos[i, 1], pt[j, 1]], [pos[i, 2], pt[j, 2]], c=['r', 'g', 'b'][j])
ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], c='yellow', s=10)
ax.axis('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
fig.savefig(f'{root}/{name}/trajectory.png')