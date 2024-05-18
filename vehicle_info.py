import os
import time

import beamngpy
from beamngpy import BeamNGpy, Scenario, Vehicle

# Instantiate BeamNGpy instance running the simulator from the given path,
# communicating over localhost:64256
bng = BeamNGpy('localhost', 64256, home='C:\\Users\\Tymon\\Documents\\BeamNG.tech.v0.30.6.0', user='C:\\Users\\Tymon\\Documents\\Projects\\beamng_test')
# Launch BeamNG.tech
bng.open()

# # Get all avaliable vehicles
# vapi = beamngpy.api.beamng.VehiclesApi(bng)
# available_vehicles = vapi.get_available()
# print('available_vehicles', available_vehicles)
# with open('available_vehicles.txt', 'w') as f:
#     f.write(str(available_vehicles))

# # Get all levels and scenarios
# sapi = beamngpy.api.beamng.ScenarioApi(bng)
# levels_scenarios = sapi.get_levels_and_scenarios()
# print('levels_scenarios', levels_scenarios)
# with open('levels_scenarios.txt', 'w') as f:
#     f.write(str(levels_scenarios))

# Get the annotation information
capi = beamngpy.api.beamng.CameraApi(bng)
info = capi.get_annotations()
print('annotation info', info)
with open('annotation_info.txt', 'w') as f:
    f.write(str(info))

quit()

model = 'pickup'
os.makedirs('vehicle_'+model, exist_ok=True)

# Create a scenario in west_coast_usa called 'example'
# scenario = Scenario('west_coast_usa', 'example')
scenario = Scenario('gridmap_v2', 'Collect Mountain')
# Create an ETK800 with the licence plate 'PYTHON'
vehicle = Vehicle('ego_vehicle', model=model, license='PYTHON')
# Add it to our scenario at this position and rotation
# scenario.add_vehicle(vehicle, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
scenario.add_vehicle(vehicle, pos=(-337.682, -491.360, 100.304), rot_quat=(0, 0, 0, 1))
# Place files defining our scenario for the simulator to read
scenario.make(bng)

# Load and start our scenario
bng.scenario.load(scenario)
bng.scenario.start()

print('keep straight now')
time.sleep(1)

vehicle.poll_sensors()
state = vehicle.state
state['cog'] = vehicle.get_center_of_gravity()
with open(os.path.sep.join(('vehicle_'+model, 'state.txt')), 'w') as f:
    f.write(str(state))

mesh = beamngpy.sensors.Mesh('mesh_sensor', bng, vehicle)
mesh.poll()
nodes = mesh.get_node_positions()
with open(os.path.sep.join(('vehicle_'+model, 'nodes.txt')), 'w') as f:
    f.write(str(nodes))

print('tuen left now')
time.sleep(5)

mesh.poll()
nodes = mesh.get_node_positions()
with open(os.path.sep.join(('vehicle_'+model, 'nodes_turnleft.txt')), 'w') as f:
    f.write(str(nodes))

bng.close()
# input('Hit enter when done...')