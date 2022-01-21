#!/usr/bin/env python
from generator import car_to_time_to_events
from scenic.domains.driving.roads import Network
from solver import ASPSolver
import argparse
import pickle
import scenic
from generator import geometry_atoms

parser = argparse.ArgumentParser(description='play the given scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

# Events:
for car, events in scenario.events.items():
    print(f'{car}:')
    for e in events:
        print(f'\t{e.withTime(e.frame)}')

params = {'map': scenario.map_path,  # scenic.simulators.carla.model
          'carla_map': scenario.map_name,  # scenic.simulators.carla.model
          'timestep': scenario.timestep,  # scenic.simulators.carla.model
          'weather': scenario.weather,  # scenic.simulators.carla.model
          'render': True,  # scenic.simulators.carla.model
          'replay_scenario': scenario
          }

print('Replay the loaded scenario...')
scenic_scenario = scenic.scenarioFromFile(
    'replay.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)

atoms = []
network = Network.fromFile(scenario.map_path)
atoms += geometry_atoms(network, scenario.intersection_uid)

event_atoms = []
car2time2events = car_to_time_to_events(scenario.events)
for car, time2events in car2time2events.items():
    for t, events in time2events.items():
        event_atoms += [f'{e.withTime(t)}' for e in events]
atoms += event_atoms

min_perceptible_time = 10  # frames
sym2val = {t: events[0].frame
           for time2events in car2time2events.values() for t, events in time2events.items()}
atoms += [f'#script(python)\n'
          f'import clingo\n'
          f'sym2val = {sym2val}\n'
          f'def lessThan(S, T):\n'
          f'  lt = sym2val[S.name] + {min_perceptible_time} < sym2val[T.name]\n'
          f'  return clingo.Number(1) if lt else clingo.Number(0)\n'
          f'def equal(S, T):\n'
          f'  eq = abs(sym2val[S.name] - sym2val[T.name]) < {min_perceptible_time}\n'
          f'  return clingo.Number(1) if eq else clingo.Number(0)\n'
          f'#end']
for s in sym2val:
    for t in sym2val:
        atoms += [f':- lessThan({s}, {t}), 0 = @lessThan({s}, {t})',
                  f':- equal({s}, {t}), 0 = @equal({s}, {t})']

solver = ASPSolver()
solver.load(scenario.rules_path)
solver.add_atoms(atoms)

model = solver.solve()

sol_names = {'violatesRule', 'violatesRightOfForRule'}
print('Violations:')
for atom in model:
    if atom.name in sol_names:
        print(f'\t{atom}')
