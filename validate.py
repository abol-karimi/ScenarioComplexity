#!/home/ak/Scenic/.venv/bin/python
from generator import car_to_time_to_events
from scenic.domains.driving.roads import Network
from solver import Solver
import argparse
import pickle
import scenic
from generator import geometry_atoms
import intersection_monitor

parser = argparse.ArgumentParser(description='validate the given scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

# Events:
for car, events in scenario.events.items():
    print(f'{car}:')
    for e in events:
        print(f'\t{e.withTime(e.frame)}')

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

# Events:
for car, events in monitor.events.items():
    print(f'{car}:')
    for e in events:
        print(f'\t{e.withTime(e.frame)}')

# atoms = []
# network = Network.fromFile(scenario.map_path)
# atoms += geometry_atoms(network, scenario.intersection_uid)

# event_atoms = []
# car2time2events = car_to_time_to_events(scenario.events)
# for car, time2events in car2time2events.items():
#     for t, events in time2events.items():
#         event_atoms += [f'{e.withTime(t)}' for e in events]
# atoms += event_atoms

# min_perceptible_time = 10  # frames
# sym2val = []
# for car, time2events in car2time2events.items():
#     for t, events in time2events.items():
#         sym2val += [(t, events[0].frame)]
# for i in range(len(sym2val)-1):
#     for j in range(i+1, len(sym2val)):
#         ti, vi = sym2val[i]
#         tj, vj = sym2val[j]
#         if abs(vi-vj) < min_perceptible_time:
#             atoms += [f'equal({ti}, {tj})', f'equal({tj}, {ti})']
#         elif vi-vj <= -min_perceptible_time:
#             atoms += [f'lessThan({ti}, {tj})']
#         else:
#             atoms += [f'lessThan({tj}, {ti})']

# solver = Solver()
# solver.load(scenario.rules_path)
# solver.add_atoms(atoms)

# model = solver.solve()

# sol_names = {'violatesRule', 'violatesRightOfForRule'}
# print('Violations:')
# for atom in model:
#     if atom.name in sol_names:
#         print(f'\t{atom}')
