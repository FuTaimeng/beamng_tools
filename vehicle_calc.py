import os
import json
import numpy as np
import matplotlib.pyplot as plt

path = 'vehicle_pickup'

with open(os.path.sep.join((path, 'nodes.txt')), 'r') as f:
    nodes = eval(f.readline())
nodes_wheels = {}
for k, v in nodes.items():
    if 'partOrigin' in v and 'wheel' in v['partOrigin']:
        nodes_wheels[k] = v
print('num nodes', len(nodes))
print('num wheel nodes', len(nodes_wheels))

def get_mass_pos(nodes):
    mass, pos = [], []
    for n in nodes.values():
        mass.append(n['mass'])
        pos.append([n['pos']['x'], n['pos']['y'], n['pos']['z']])
    mass = np.array(mass)
    pos = np.array(pos)
    return mass, pos

mass, pos = get_mass_pos(nodes)
mass_wheels, pos_wheels = get_mass_pos(nodes_wheels)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

color = np.array(['b'] * len(nodes))
ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], s=mass*2, c=color, picker=True)

front_left = np.logical_and(pos_wheels[:, 0] > 0, pos_wheels[:, 1] <= 0)
front_right = np.logical_and(pos_wheels[:, 0] <= 0, pos_wheels[:, 1] <= 0)
rear_left = np.logical_and(pos_wheels[:, 0] > 0, pos_wheels[:, 1] > 0)
rear_right = np.logical_and(pos_wheels[:, 0] <= 0, pos_wheels[:, 1] > 0)

def get_center_radius(pos):
    center = np.mean(pos, axis=0)
    low_z = np.min(pos[:, 2])
    radius = center[2] - low_z
    return center.tolist(), radius

res = {}
res['FL_center'], res['FL_radius'] = get_center_radius(pos_wheels[front_left])
res['FR_center'], res['FR_radius'] = get_center_radius(pos_wheels[front_right])
res['RL_center'], res['RL_radius'] = get_center_radius(pos_wheels[rear_left])
res['RR_center'], res['RR_radius'] = get_center_radius(pos_wheels[rear_right])

for i in ['FL', 'FR', 'RL', 'RR']:
    x, y, z = res[f'{i}_center']
    r = res[f'{i}_radius']
    ax.scatter(x, y, z, color='g', s=50)
    ax.plot([x, x], [y, y], [z, z-r], color='r')

with open(os.path.sep.join((path, 'state.txt')), 'r') as f:
    state = eval(f.readline())
vpos = np.array(state['pos'])
cog = np.array(state['cog']) - vpos
ax.scatter(cog[0], cog[1], cog[2], s=[50], c=['r'])
res['COG'] = cog.tolist()

ax.scatter(cog[0], cog[1], cog[2], color='r', s=100)

def calc_inertia_mat(pos, mass, cog):
    I = np.zeros((3, 3))
    for i in range(len(pos)):
        x, y, z = pos[i] - cog
        m = mass[i]
        I_pt = np.array([
            [m*(y*y+z*z), -m*x*y, -m*x*z],
            [-m*y*x, m*(x*x+z*z), -m*y*z],
            [-m*z*x, -m*z*y, m*(x*x+y*y)]
        ])
        I += I_pt
    return I

res['FL_I'] = calc_inertia_mat(pos_wheels[front_left], mass_wheels[front_left], res['FL_center'])[0, 0]
res['FR_I'] = calc_inertia_mat(pos_wheels[front_right], mass_wheels[front_right], res['FR_center'])[0, 0]
res['RL_I'] = calc_inertia_mat(pos_wheels[rear_left], mass_wheels[rear_left], res['RL_center'])[0, 0]
res['RR_I'] = calc_inertia_mat(pos_wheels[rear_right], mass_wheels[rear_right], res['RR_center'])[0, 0]
res['I'] = calc_inertia_mat(pos, mass, cog).tolist()

res['mass'] = mass.tolist()
res['mass_pts'] = (pos - cog).tolist()

with open(os.path.sep.join((path, 'nodes_turnleft.txt')), 'r') as f:
    nodes = eval(f.readline())
nodes_wheels = {}
for k, v in nodes.items():
    if 'partOrigin' in v and 'wheeldata_F' in v['partOrigin']:
        nodes_wheels[k] = v

_, pos_wheels = get_mass_pos(nodes_wheels)
front_left = np.logical_and(pos_wheels[:, 0] > 0, pos_wheels[:, 1] <= 0)
pos_wheels = pos_wheels[front_left]
ax.scatter(pos_wheels[:, 0], pos_wheels[:, 1], pos_wheels[:, 2], color='purple', s=10)

svd = np.linalg.svd(pos_wheels.T - np.mean(pos_wheels.T, axis=1, keepdims=True))
n = svd[0][:, -1]
x, y, z = np.mean(pos_wheels.T, axis=1)
ax.plot([x, x+n[0]], [y, y+n[1]], [z, z+n[2]], color='purple')

ang = np.arccos(np.abs(n[0]))
# print(n, np.linalg.norm(n), ang*180/np.pi)
res['max_turn_rad'] = ang

print(res)
with open(os.path.sep.join((path, 'vehicle_config.txt')), 'w') as f:
    f.write(json.dumps(res))

ax.axis('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()
