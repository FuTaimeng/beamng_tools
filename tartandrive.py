import cv2
import torch
import numpy as np
import pypose as pp
import open3d as o3d
import matplotlib.pyplot as plt

data = torch.load('20210826_34.pt')

st = 20
end = len(data['observation']['state']) - 10
N = end - st

pose = data['observation']['state'][st:end, 0:7]
pose = pp.SE3(pose)

rgbmap = data['observation']['rgbmap'][st:end]
rgbmap = rgbmap.permute(0, 2, 3, 1)

heightmap = data['observation']['heightmap'][st:end]
heightmap = heightmap.squeeze(1)

image = data['observation']['image_rgb'][st:end]
image = image.permute(0, 2, 3, 1)

mask = torch.logical_and(torch.sum(rgbmap, dim=-1) > 0.1, heightmap < 3)
# heightmap[~mask] = 0
# mask = torch.ones(heightmap.shape, dtype=bool)
# mask[:, :, :32] = 0

# for i in range(N):
#     img = (heightmap[i].numpy()/4*255).astype(np.uint8)
#     cv2.imwrite(f'drive/height/{i:0>6d}.png', img)
#     img = (rgbmap[i].numpy()*255).astype(np.uint8)
#     cv2.imwrite(f'drive/rgb/{i:0>6d}.png', img)
#     img = (image[i].numpy()*255).astype(np.uint8)
#     cv2.imwrite(f'drive/image/{i:0>6d}.png', img)
#     # img = cv2.resize(img, None, fx=4, fy=4)
#     # cv2.imshow('rgbmap', img)
#     # cv2.waitKey(100)

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

c = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 0, 1), (0, 1, 1)]

fig = plt.figure('cloud')
ax = fig.add_subplot(projection='3d')
# for i in range(5):
#     w_points_np = w_points[50+i*5].numpy().reshape(-1, 3)
#     points_col = rgbmap[50+i*5].numpy().reshape(-1, 3).astype(float) / 255
#     ax.scatter(w_points_np[:, 0], w_points_np[:, 1], w_points_np[:, 2], '.', c=c[i])
# pos = pose.translation().numpy()
# ax.plot(pos[:, 0], pos[:, 1], pos[:, 2])
# for i in range(N):
#     pt = torch.tensor([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=torch.float32)
#     pt = pose[i] @ pt
#     for j in range(3):
#         plt.plot([pos[i, 0], pt[j, 0]], [pos[i, 1], pt[j, 1]], [pos[i, 2], pt[j, 2]], c=['r', 'g', 'b'][j])
# ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], c='yellow', s=10)
# ax.axis('equal')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.show()

# points, colors = [], []
# for i in range(5):
#     idx = 20 + i*5
#     pts = w_points[50+i*5][mask[50+i*5]].numpy().reshape(-1, 3)
#     # pts = w_points[50+i*5].numpy().reshape(-1, 3)
#     col = np.zeros_like(pts)
#     col[:] = c[i]
#     points.append(pts)
#     colors.append(col)
# points = np.concatenate(points, axis=0)
# colors = np.concatenate(colors, axis=0)
points = w_points[mask].numpy()
colors = rgbmap[mask].numpy()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.io.write_point_cloud('drive/cloud.ply', pcd, write_ascii=False)

# fig = plt.figure('cloud')
# ax = fig.add_subplot(projection='3d')
# w_points_np = w_points.numpy().reshape(-1, 3)
# points_col = rgbmap.reshape(-1, 3).astype(float) / 255
# ax.scatter(w_points_np[:, 0], w_points_np[:, 1], w_points_np[:, 2], '.', c=points_col)
# ax.axis('equal')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.show()
# plt.close(fig)

depth_bit = 16
depth_type_max = 2**depth_bit - 1
depth_type = np.uint8 if depth_bit==8 else np.uint16

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

        swap = np.argmin(np.stack([self.height[ij[:, 0], ij[:, 1]], points[:, 2]]), axis=0)
        swap[self.count[ij[:, 0], ij[:, 1]] == 0] = 1
        swap = swap.astype(bool)

        self.count[ij[:, 0], ij[:, 1]] = 1
        self.height[ij[:, 0], ij[:, 1]] = np.where(swap, points[:, 2], self.height[ij[:, 0], ij[:, 1]])
        self.rgb[ij[:, 0], ij[:, 1]] = np.where(swap[..., np.newaxis], colors, self.rgb[ij[:, 0], ij[:, 1]])

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
        height_img = np.zeros(self.height.shape, dtype=depth_type)
        height_img[mask] = np.round((height - min_h) / (max_h - min_h) * (depth_type_max-1) + 1)

        rgb_img = np.zeros(self.rgb.shape, dtype=np.uint8)
        rgb_img[mask] = np.round(self.rgb[mask] / count[..., np.newaxis] * 255)

        kernel = np.array([[1,2,1],[2,0,2],[1,2,1]], dtype=np.float32)
        kernel /= np.sum(kernel)
        for i in range(5):
            height_blur = cv2.filter2D(height_img, -1, kernel)
            rgb_blur = cv2.filter2D(rgb_img, -1, kernel)
            height_img[~mask] = height_blur[~mask]
            rgb_img[~mask] = rgb_blur[~mask]

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
        points, colors, segments = [], [], []
        for block in self.blocks.values():
            pts, rgb = block.get_points()
            points.append(pts)
            colors.append(rgb)
        points = np.concatenate(points, axis=0)
        colors = np.concatenate(colors, axis=0)
        return points, colors
    
    def get_images(self):
        heights, colors, segments = [], [], []
        infos = {}
        for i, block in enumerate(self.blocks.values()):
            height_img, rgb_img, seg_img, h_range = block.get_image()
            heights.append(height_img)
            colors.append(rgb_img)
            bID = tuple(map(int, block.bID))
            infos[i] = {'bID':bID, 'h_range':h_range}
        return heights, colors, segments, infos
    
dm = Map(0.08)
for i in range(N):
    points = w_points[i].numpy().reshape(-1, 3)
    colors = rgbmap[i].numpy().reshape(-1, 3)
    dm.add_points(points, colors)

points, colors = dm.get_pointcloud()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.io.write_point_cloud('drive/cloud2.ply', pcd, write_ascii=False)

print('done')