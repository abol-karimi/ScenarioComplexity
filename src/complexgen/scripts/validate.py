#!/usr/bin/env python3.8
from complexgen.core.generator import car_to_time_to_events
from scenic.domains.driving.roads import Network
from complexgen.core.solver import ASPSolver
import argparse
import pickle
import scenic
from complexgen.core.generator import geometry_atoms
import src.complexgen.core.intersection_monitor as intersection_monitor

parser = argparse.ArgumentParser(description='validate the given scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

event2frame = {}
for events in scenario.events.values():
    for e in events:
        event2frame[e.withTime('')] = e.frame

monitor = intersection_monitor.Monitor()

params = {'map': scenario.map_path,  # scenic.simulators.carla.model
          'carla_map': scenario.map_name,  # scenic.simulators.carla.model
          'timestep': scenario.timestep,  # scenic.simulators.carla.model
          'weather': scenario.weather,  # scenic.simulators.carla.model
          'render': False,  # scenic.simulators.carla.model
          'replay_scenario': scenario,
          'stop_speed_threshold': 0.5,  # meters/seconds
          'event_monitor': monitor
          }

print('Compute events of the loaded scenario...')
scenic_scenario = scenic.scenarioFromFile(
    'validate.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)

for events in monitor.events.values():
    for e in events:
        if e.name == 'signaledAtForkAtTime':
            continue
        e_id = e.withTime('')
        t0 = event2frame[e_id]
        t1 = e.frame
        print(f'{e_id:60} {t0} --> {t1}')
