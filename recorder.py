import os
import cv2
import time
import json
import torch
import datetime
import numpy as np
import pypose as pp

import beamngpy
from beamngpy import BeamNGpy, Scenario, Vehicle

dataroot = 'data_stop'
os.makedirs(dataroot, exist_ok=True)

scenario_name = 'gridmap_v2'
model_name = 'pickup'
record_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

record_param = {
    'scenario': scenario_name,
    'model': model_name,
    'time': record_time
}
with open(os.path.sep.join((dataroot, 'record_config.txt')), 'w') as f:
    json.dump(record_param, f)

USERPATH = fr'{os.getenv("LOCALAPPDATA")}/BeamNG.drive'
print(USERPATH)
# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\Tymon\\Documents\\BeamNG.tech.v0.30.6.0', user=USERPATH)
# Launch BeamNG.tech
bng.open(extensions=['util/gameEngineCode'])

# Create a scenario in west_coast_usa called 'example'
# scenario = Scenario('west_coast_usa', 'example')
scenario = Scenario(scenario_name, 'Collect Mountain')
# scenario = Scenario('italy', 'Mountain')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model=model_name, license='PYTHON', extensions=['vehicleEngineCode'])
# vehicle = Vehicle('ego_vehicle', model='autobello', license='PYTHON', extensions=['vehicleEngineCode'])
# Add it to our scenario at this position and rotation
# scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
scenario.add_vehicle(vehicle, pos=(-337.682, -491.360, 100.304), rot_quat=(0, 0, 0, 1))
# scenario.add_vehicle(vehicle, pos=(-279.385, -503.356, 100.340), rot_quat=(0, 0, 0, 1))
# scenario.add_vehicle(vehicle, pos=(-1256.066, -1104.325, 589.563), rot_quat=(0, 0, 0, 1))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.scenario.load(scenario)
bng.scenario.start()

width, height = 640, 480
fov_y = 70
pos = (0, -2, 2)
dir = np.array((0.0,-1.0,-0.5))
dir /= np.linalg.norm(dir)
up = np.array((0.0,-0.5,1.0))
up /= np.linalg.norm(up)
left = np.cross(up, dir)
assert np.linalg.norm(left - np.array((1.0,0.0,0.0))) < 1e-5
dir = dir.tolist()
up = up.tolist()
near_far_planes = (0.05, 100)

cam_param = {
    'resolution': (width, height),
    'fov_y': fov_y,
    'dir': dir,
    'up': up,
    'near_far_planes': near_far_planes
}
# cam_param['trans'] = [0.5502, -1.1516,  1.6490]
# with open(os.path.sep.join((dataroot, 'cam_param.txt')), 'w') as f:
#     json.dump(cam_param, f)
# quit()

cam = beamngpy.sensors.Camera(
    'cam', bng, vehicle,
    requested_update_time=0.1,
    update_priority=1.0,
    pos=pos, dir=dir, up=up,
    resolution=(width, height),
    field_of_view_y=fov_y,
    near_far_planes=near_far_planes,
    is_using_shared_memory=True,
    is_streaming=True,
    # is_snapping_desired=True,
)

electrics = beamngpy.sensors.Electrics()
vehicle.attach_sensor('electrics', electrics)

poses, controls = [], []
cogs, cam_poses = [], []
wheel_speeds = []
timestamps = []
idx = -1

os.makedirs(os.path.sep.join((dataroot, 'rgb')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'seg')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'dep')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'depth')), exist_ok=True)

time.sleep(1)

def process_depth(binary):
    depth = np.frombuffer(binary['depth'], dtype=np.float32) * near_far_planes[1]
    reshaped_data = depth.reshape(height, width)
    return reshaped_data

def getWheelSpeeds(veh):
    data = dict(type='GetWheelSpeeds')
    response = veh.connection.send(data)
    return response.recv()

while True:
    ts = time.perf_counter()
    vehicle.poll_sensors()
    t1 = time.perf_counter()
    whspeed = getWheelSpeeds(vehicle)
    t2 = time.perf_counter()
    raw_data = cam.poll_raw()
    t3 = time.perf_counter()
    cog = vehicle.get_center_of_gravity()
    t4 = time.perf_counter()
    cam_pos, cam_dir = cam.get_position(), cam.get_direction()
    t5 = time.perf_counter()
    timestamps.append([ts, t1, t2, t3, t4, t5])
    idx += 1

    print('times', t1-ts, t2-t1, t3-t2)
    
    data = cam._binary_to_image(raw_data)
    depth = process_depth(raw_data)
    
    state = vehicle.state
    pos = state['pos']
    rot = state['rotation']
    rot = pp.SO3(rot).Inv().tolist()
    vel = state['vel']
    poses.append(pos + rot + vel)

    estate = vehicle.sensors['electrics']
    controls.append([estate['throttle_input'], estate['brake_input'], estate['steering_input']])

    wheel_speeds.append([whspeed['FL'], whspeed['FR'], whspeed['RL'], whspeed['RR']])

    cogs.append(cog)
    cam_poses.append(cam_pos + cam_dir)

    rgb = cv2.cvtColor(np.array(data['colour']), cv2.COLOR_RGB2BGR)
    seg = cv2.cvtColor(np.array(data['annotation']), cv2.COLOR_RGB2BGR)
    dep = np.array(data['depth']).astype(np.uint8)

    cv2.imwrite(os.path.sep.join((dataroot, 'rgb', '{:0>6}.png'.format(idx))), rgb)
    cv2.imwrite(os.path.sep.join((dataroot, 'seg', '{:0>6}.png'.format(idx))), seg)
    cv2.imwrite(os.path.sep.join((dataroot, 'dep', '{:0>6}.png'.format(idx))), dep)
    np.save(os.path.sep.join((dataroot, 'depth', '{:0>6}.npy'.format(idx))), depth)

    cv2.imshow('rgb', rgb)
    cv2.imshow('seg', seg)
    cv2.imshow('dep', dep)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    # pos = torch.tensor(pos)
    # cam_pos = torch.tensor(cam.get_position())
    # cam_dir = torch.tensor(cam.get_direction())
    # print('pos', pos, 'cam_pos', cam_pos)
    # pose = pp.SE3(poses[-1])
    # rel_pos = pose.Inv() @ cam_pos
    # rel_dir = pose.rotation() @ cam_dir
    # print('rel_pos', rel_pos, 'rel_dir', rel_dir)
    # delta_pos = cam_pos - pos
    # print('delta_pose', delta_pos, 'test', pose.rotation() @ delta_pos)

    te = time.perf_counter()
    # print('time', te-ts)
    if te-ts < 0.1:
        while True:
            te = time.perf_counter()
            if te - ts >= 0.1:
                break
    else:
        print('timeout!', idx, te-ts)

poses = np.array(poses)
controls = np.array(controls)
cogs = np.array(cogs)
cam_poses = np.array(cam_poses)
wheel_speeds = np.array(wheel_speeds)
timestamps = np.array(timestamps)
np.savetxt(os.path.sep.join((dataroot, 'pose.txt')), poses)
np.savetxt(os.path.sep.join((dataroot, 'control.txt')), controls)
np.savetxt(os.path.sep.join((dataroot, 'cog.txt')), cogs)
np.savetxt(os.path.sep.join((dataroot, 'cam_pose.txt')), cam_poses)
np.savetxt(os.path.sep.join((dataroot, 'wheel_speed.txt')), wheel_speeds)
np.savetxt(os.path.sep.join((dataroot, 'timestamp.txt')), timestamps)

def cam_extrinsic_trans():
    trans = []
    for i in range(idx-10, idx):
        pose = pp.SE3(poses[i, :7])
        cam_pos = torch.tensor(cam_poses[i, :3], dtype=torch.float32)
        cam_pos_wrt_vehicle = pose.Inv() @ cam_pos
        trans.append(cam_pos_wrt_vehicle)
    trans = torch.stack(trans)
    return torch.mean(trans, dim=0).tolist()

cam_param['trans'] = cam_extrinsic_trans()
with open(os.path.sep.join((dataroot, 'cam_param.txt')), 'w') as f:
    json.dump(cam_param, f)

print('total frames', idx)

def print_timecost(ts):
    print('timecost:')
    print('\tvehicle.poll_sensors()', np.mean(ts[:, 1] - ts[:, 0]))
    print('\tgetWheelSpeeds(vehicle)', np.mean(ts[:, 2] - ts[:, 1]))
    print('\tcam.poll_raw()', np.mean(ts[:, 3] - ts[:, 2]))
    print('\tvehicle.get_center_of_gravity()', np.mean(ts[:, 4] - ts[:, 3]))
    print('\tcam.get_position(), cam.get_direction()', np.mean(ts[:, 5] - ts[:, 4]))

print_timecost(timestamps)
