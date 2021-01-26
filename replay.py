#!/home/ak/Scenic/.venv/bin/python
import argparse
from generator import frame_to_ruletime
import pickle
import scenic

parser = argparse.ArgumentParser(description='play the given scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

for car in scenario.events:
    for event in scenario.events[car]:
        ruletime = frame_to_ruletime(event.frame, scenario.timestep)
        print(event.withTime(ruletime))

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_id': scenario.intersection_id,
          'maneuver_id': scenario.maneuver_id,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': False}

print('Replay the loaded scenario...')
params['trajectory'] = scenario.trajectory
params['blueprints'] = scenario.blueprints
scenic_scenario = scenic.scenarioFromFile(
    'replay.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)
