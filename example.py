import os
import cv2
import numpy as np

import beamngpy
from beamngpy import BeamNGpy, Scenario, Vehicle

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\Tymon\\Documents\\BeamNG.tech.v0.30.6.0', user='C:\\Users\\Tymon\\Documents\\Projects\\beamng_test')
# Launch BeamNG.tech
bng.open()
# Create a scenario in west_coast_usa called 'example'
scenario = Scenario('west_coast_usa', 'example')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model='etk800', license='PYTHON')
# Add it to our scenario at this position and rotation
scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.scenario.load(scenario)
bng.scenario.start()

cam = beamngpy.sensors.Camera(
    'cam', bng, vehicle,
    pos=(0,-2,2), dir=(0,-1,-1), up=(0,-1,1),
    resolution=(640,480),
    near_far_planes=(0.1,12.75)
)

# Make the vehicle's AI span the map
vehicle.ai.set_mode('span')

dataroot = 'data'
poses = []

for idx in range(100):
    data = cam.poll()
    if data is None or data['colour'] is None:
        continue

    pos = cam.get_position()
    dir = cam.get_direction()
    up = (0,-1,1)
    poses.append(list(pos + dir + up))

    rgb = cv2.cvtColor(np.array(data['colour']), cv2.COLOR_RGB2BGR)
    seg = cv2.cvtColor(np.array(data['annotation']), cv2.COLOR_RGB2BGR)
    dep = np.array(data['depth']).astype(np.uint8)

    # cv2.imwrite(os.path.sep.join((dataroot, 'rgb', '{:0>6}.png'.format(idx))), rgb)
    # cv2.imwrite(os.path.sep.join((dataroot, 'seg', '{:0>6}.png'.format(idx))), seg)
    # cv2.imwrite(os.path.sep.join((dataroot, 'dep', '{:0>6}.png'.format(idx))), dep)

    cv2.imshow('rgb', rgb)
    cv2.imshow('seg', seg)
    cv2.imshow('dep', dep)
    cv2.waitKey(1)

poses = np.array(poses)
np.savetxt(os.path.sep.join((dataroot, 'poses.txt')), poses)

bng.close()
# input('Hit enter when done...')