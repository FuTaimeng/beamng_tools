import os
import beamngpy.sensors
import cv2
import time
import json
import torch
import datetime
import numpy as np
import pypose as pp

import beamngpy
from beamngpy import BeamNGpy, Scenario, Vehicle

from spawn_points import spawn_points

mode = ['manual', 'ai-span'][1]
spawn_pt = spawn_points['derby'][1]

scenario_name = spawn_pt['scenario_name']
level_name = spawn_pt['point_name']
model_name = 'pickup'
record_time = datetime.datetime.now().strftime("%y-%m-%d-%H-%M-%S")

scenario_name_display = scenario_name.replace('_', '-')
dataroot = f'data/{mode}_{scenario_name_display}_{level_name}_{model_name}_{record_time}'
os.makedirs(dataroot, exist_ok=True)

record_param = {
    'mode': mode,
    'scenario': scenario_name,
    'level': level_name,
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
scenario = Scenario(scenario_name, level_name)
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('vehicle0', model=model_name, license='TYMON', extensions=['vehicleEngineCode'])
# Add it to our scenario at this position and rotation
position = spawn_pt['position']
rotation = pp.euler2SO3(spawn_pt['euler']).tolist()
scenario.add_vehicle(vehicle, pos=position, rot_quat=rotation)
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

damage = beamngpy.sensors.Damage()
vehicle.attach_sensor('damage', damage)

poses = []
controls = []
gears = []
wheel_speeds = []
timestamps = []

idx = -1
low_speed_cnt = 0

os.makedirs(os.path.sep.join((dataroot, 'rgb')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'seg')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'dep')), exist_ok=True)
os.makedirs(os.path.sep.join((dataroot, 'depth')), exist_ok=True)

time.sleep(2)

def cam_extrinsic_trans():
    vehicle.poll_sensors()
    cam_pos = cam.get_position()

    state = vehicle.state
    pos = state['pos']
    rot = state['rotation']
    rot = pp.SO3(rot).Inv().tolist()
    pose = pp.SE3(pos + rot)

    cam_pos = torch.tensor(cam_pos, dtype=torch.float32)

    cam_pos_wrt_vehicle = pose.Inv() @ cam_pos
    return cam_pos_wrt_vehicle.tolist()

cam_param['trans'] = cam_extrinsic_trans()
with open(os.path.sep.join((dataroot, 'cam_param.txt')), 'w') as f:
    json.dump(cam_param, f)

if mode == 'ai-span':
    vehicle.ai.set_aggression(0.6)
    vehicle.ai.drive_in_lane(False)
    vehicle.ai.set_mode('span')

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

    # if damage['damage'] > 0.0:
    #     break

    idx += 1

    timestamps.append([ts, t1, t2, t3])
    # print('sensor times', t1-ts, t2-t1, t3-t2)
    
    state = vehicle.state
    pos = state['pos']
    rot = state['rotation']
    rot = pp.SO3(rot).Inv().tolist()
    vel = state['vel']
    poses.append(pos + rot + vel)

    estate = vehicle.sensors['electrics']
    controls.append([estate['throttle_input'], estate['brake_input'], estate['steering_input']])
    gears.append(int(estate['gear_index']))

    wheel_speeds.append([whspeed['FL'], whspeed['FR'], whspeed['RL'], whspeed['RR']])

    data = cam._binary_to_image(raw_data)
    depth = process_depth(raw_data)

    rgb = cv2.cvtColor(np.array(data['colour']), cv2.COLOR_RGB2BGR)
    seg = cv2.cvtColor(np.array(data['annotation']), cv2.COLOR_RGB2BGR)
    dep = np.array(data['depth']).astype(np.uint8)

    cv2.imwrite(os.path.sep.join((dataroot, 'rgb', '{:0>6}.png'.format(idx))), rgb)
    cv2.imwrite(os.path.sep.join((dataroot, 'seg', '{:0>6}.png'.format(idx))), seg)
    cv2.imwrite(os.path.sep.join((dataroot, 'dep', '{:0>6}.png'.format(idx))), dep)
    np.save(os.path.sep.join((dataroot, 'depth', '{:0>6}.npy'.format(idx))), depth)

    cv2.imshow('rgb', rgb)
    # cv2.imshow('seg', seg)
    # cv2.imshow('dep', dep)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    if np.linalg.norm(vel) < 1.0:
        low_speed_cnt += 1
    else:
        low_speed_cnt = 0
    if low_speed_cnt >= 50:
        break
    if idx >= 3000:
        break

    te = time.perf_counter()
    # print('step time', te-ts)

    if te-ts < 0.1:
        while True:
            te = time.perf_counter()
            if te - ts >= 0.1:
                break
    else:
        print('timeout!', idx, te-ts)

poses = np.array(poses)
controls = np.array(controls)
gears = np.array(gears)
wheel_speeds = np.array(wheel_speeds)
timestamps = np.array(timestamps)
np.savetxt(os.path.sep.join((dataroot, 'pose.txt')), poses)
np.savetxt(os.path.sep.join((dataroot, 'control.txt')), controls)
np.savetxt(os.path.sep.join((dataroot, 'gears.txt')), gears)
np.savetxt(os.path.sep.join((dataroot, 'wheel_speed.txt')), wheel_speeds)
np.savetxt(os.path.sep.join((dataroot, 'timestamp.txt')), timestamps)

print('total frames', idx)

def print_timecost(ts):
    print('timecost:')
    print('\tvehicle.poll_sensors()', np.mean(ts[:, 1] - ts[:, 0]))
    print('\tgetWheelSpeeds(vehicle)', np.mean(ts[:, 2] - ts[:, 1]))
    print('\tcam.poll_raw()', np.mean(ts[:, 3] - ts[:, 2]))

print_timecost(timestamps)
