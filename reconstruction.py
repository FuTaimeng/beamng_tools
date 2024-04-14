import os
import sys
import cv2
import time
import json
import torch
import numpy as np
import pypose as pp
import open3d as o3d

if len(sys.argv) >= 2:
    dataroot = sys.argv[1]
else:
    dataroot = 'data/ai-span_derby_grass-asphalt-arena_pickup_24-04-13-20-48-26'
poses = np.loadtxt(os.path.sep.join((dataroot, 'pose.txt')))

old_mode = False
if not old_mode:
    grid_length = 0.02
else:
    grid_length = 0.2
block_size = 1000
start_frame = 0
end_frame = len(poses)

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
K = np.array([
    f, 0, cx,
    0, f, cy,
    0,  0,  1
], dtype=float).reshape(3,3)
K_inv = np.linalg.inv(K)
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


class MapBlock:
    def __init__(self, bID, grid_length, block_size=1000):
        self.bID = bID
        self.grid_length = grid_length
        self.block_size = block_size
        
        block_length = grid_length * block_size
        self.start_pos = np.array([bID[0]*block_length, bID[1]*block_length], dtype=float)

        xy = np.linspace(self.start_pos+grid_length/2, self.start_pos+(block_length-grid_length/2), block_size)
        X, Y = np.meshgrid(xy[:, 0], xy[:, 1], indexing='ij')
        self.grid = np.stack([X, Y], axis=-1)

        self.count = np.zeros((block_size, block_size), dtype=int)
        self.height = np.zeros((block_size, block_size), dtype=float)
        self.rgb = np.zeros((block_size, block_size, 3), dtype=float)

    def xy2ij(self, xy):
        return np.floor((xy - self.start_pos) / self.grid_length).astype(int)
    
    def add_points(self, points, colors):
        ij = self.xy2ij(points[:, :2])
        assert np.all(ij < self.block_size) and np.all(ij >= 0)

        self.count[ij[:, 0], ij[:, 1]] += 1
        self.height[ij[:, 0], ij[:, 1]] += points[:, 2]
        self.rgb[ij[:, 0], ij[:, 1]] += colors

    def get_points(self):
        mask = self.count > 0
        grid = self.grid[mask]
        count = self.count[mask]
        height = self.height[mask] / count
        pts = np.concatenate((grid, height[..., np.newaxis]), axis=-1)
        rgb = self.rgb[mask] / count[..., np.newaxis]
        return pts, rgb
    
    def get_image(self):
        mask = self.count > 0
        count = self.count[mask]
        height = self.height[mask] / count
        min_h, max_h = np.min(height), np.max(height)
        if max_h - min_h < 0.1:
            min_h = max_h - 0.1
        height_img = np.zeros(self.height.shape, dtype=np.uint8)
        height_img[mask] = np.round((height - min_h) / (max_h - min_h) * 254 + 1)
        rgb_img = np.zeros(self.rgb.shape, dtype=np.uint8)
        rgb_img[mask] = np.round(self.rgb[mask] / count[..., np.newaxis] * 255)
        kernel = np.array([[1,2,1],[2,0,2],[1,2,1]], dtype=np.float32)
        kernel /= np.sum(kernel)
        for i in range(5):
            height_blur = cv2.filter2D(height_img, -1, kernel)
            rgb_blur = cv2.filter2D(rgb_img, -1, kernel)
            height_img[~mask] = height_blur[~mask]
            rgb_img[~mask] = rgb_blur[~mask]
        # blur = cv2.GaussianBlur(height_img, (7,7), 0)
        # inpaint_mask = np.logical_and(height_img == 0, blur != 0).astype(np.uint8)
        # height_img = cv2.inpaint(height_img, inpaint_mask, 5, cv2.INPAINT_TELEA)
        # rgb_img = cv2.inpaint(rgb_img, inpaint_mask, 5, cv2.INPAINT_TELEA)
        return height_img, rgb_img, (min_h, max_h)

class Map:
    def __init__(self, grid_length, block_size=1000):
        self.grid_length = grid_length
        self.block_size = block_size
        self.block_length = grid_length * block_size

        self.blocks = {}

    def xy2block(self, xy):
        return np.floor(xy / self.block_length).astype(int)

    def add_points(self, points, colors):
        ij = self.xy2block(points[:, :2])
        bID = np.unique(ij, axis=0)
        bID = list(map(tuple, bID))
        for b in bID:
            if b not in self.blocks:
                self.blocks[b] = MapBlock(b, self.grid_length, self.block_size)
            mask = np.logical_and(ij[:, 0] == b[0], ij[:, 1] == b[1])
            self.blocks[b].add_points(points[mask], colors[mask])
            
    def get_pointcloud(self):
        points, colors = [], []
        for block in self.blocks.values():
            pts, rgb = block.get_points()
            points.append(pts)
            colors.append(rgb)
        points = np.concatenate(points, axis=0)
        colors = np.concatenate(colors, axis=0)
        return points, colors
    
    def get_images(self):
        heights, colors = [], []
        infos = {}
        for i, block in enumerate(self.blocks.values()):
            height_img, rgb_image, h_range = block.get_image()
            heights.append(height_img)
            colors.append(rgb_image)
            bID = tuple(map(int, block.bID))
            infos[i] = {'bID':bID, 'h_range':h_range}
        return heights, colors, infos


terrian_map = Map(grid_length, block_size)

for idx in range(start_frame, end_frame):
    print('\rprocessing {}/{} ...'.format(idx, end_frame), end='')

    pose = pp.SE3(poses[idx, :7])
    rgb = cv2.imread(os.path.sep.join((dataroot, 'rgb', '{:0>6}.png'.format(idx))), cv2.IMREAD_COLOR)
    seg = cv2.imread(os.path.sep.join((dataroot, 'seg', '{:0>6}.png'.format(idx))), cv2.IMREAD_COLOR)
    depth = np.load(os.path.sep.join((dataroot, 'depth', '{:0>6}.npy'.format(idx))))

    rgb = rgb.astype(float) / 255
    seg = seg.astype(float) / 255
    depth = depth.astype(float)

    # mask = np.zeros(resolution[0]*resolution[1], dtype=bool)
    # mask[::10] = True
    mask = np.ones(resolution[0]*resolution[1], dtype=bool)
    mask = np.logical_and(mask, depth.reshape(-1) < near_far_planes[1]/10)

    u_lin = np.linspace(0, resolution[0]-1, resolution[0])
    v_lin = np.linspace(0, resolution[1]-1, resolution[1])
    u, v = np.meshgrid(u_lin, v_lin, indexing='xy')
    uv1 = np.transpose(np.stack([u, v, np.ones_like(u)]), axes=(1,2,0))

    uv1 = uv1.reshape(-1,3,1)[mask, :, :]
    depth = depth.reshape(-1,1)[mask, :]
    rgb = rgb.reshape(-1,3)[mask, :]
    seg = seg.reshape(-1,3)[mask, :]

    pts_cam = (K_inv.reshape(1,3,3) @ uv1).squeeze(-1) * depth
    pts_cam = torch.tensor(pts_cam, dtype=torch.float32)

    pts_veh = extrinsic @ pts_cam

    height_mask = pts_veh[:, 2] < 1.0
    pts_veh = pts_veh[height_mask]
    rgb = rgb[height_mask]

    pts_world = (pose @ pts_veh).numpy()

    terrian_map.add_points(pts_world, rgb)

if not old_mode:
    t0 = time.time()
    heights, colors, infos = terrian_map.get_images()
    t1 = time.time()
    print('get_images time:', t1-t0)
    if os.path.isdir(os.path.sep.join((dataroot, 'map'))):
        os.system('rm -r '+os.path.sep.join((dataroot, 'map')))
    os.makedirs(os.path.sep.join((dataroot, 'map', 'height')), exist_ok=True)
    os.makedirs(os.path.sep.join((dataroot, 'map', 'color')), exist_ok=True)
    infos.update({'num_block':len(heights), 'grid_length':grid_length, 'block_size':block_size})
    with open(os.path.sep.join((dataroot, 'map', 'info.txt')), 'w') as f:
        json.dump(infos, f)
    for i in range(len(heights)):
        cv2.imwrite(os.path.sep.join((dataroot, 'map', 'height', f'{i:0>6d}.png')), heights[i])
        cv2.imwrite(os.path.sep.join((dataroot, 'map', 'color', f'{i:0>6d}.png')), colors[i])

    points, colors = terrian_map.get_pointcloud()
    points, colors = points[::100], colors[::100]
    points_colors = np.concatenate((points, colors), axis=-1)
    # np.save(os.path.sep.join((dataroot, 'cloud.npy')), points_colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(os.path.sep.join((dataroot, 'map', 'cloud.ply')), pcd, write_ascii=False)
