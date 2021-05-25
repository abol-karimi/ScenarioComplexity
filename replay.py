#!/home/ak/Scenic/.venv/bin/python
from scenic.domains.driving.roads import Network
from solver import Solver
import argparse
from generator import frame_to_ruletime
import pickle
import scenic
from generator import geometry_atoms, frame_to_ruletime

parser = argparse.ArgumentParser(description='play the given scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

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
for car in scenario.events.keys():
    for event in scenario.events[car]:
        ruletime = frame_to_ruletime(event.frame, scenario.timestep)
        atom = event.withTime(ruletime)
        event_atoms.append(atom)
atoms += event_atoms

solver = Solver()
solver.load(scenario.rules_path)
solver.add_atoms(atoms)

model = solver.solve()

print('Events:')
for atom in event_atoms:
    print(f'\t{atom}')

sol_names = {'violatesRule', 'violatesRightOfForRule'}
print('Violations:')
for atom in model:
    if atom.name in sol_names:
        print(f'\t{atom}')
