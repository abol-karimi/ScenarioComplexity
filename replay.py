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

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_uid': scenario.intersection_uid,
          'maneuver_uid': scenario.maneuver_uid,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': True}

print('Replay the loaded scenario...')
params['trajectory'] = scenario.trajectory
params['blueprints'] = scenario.blueprints
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

max_ruletime = frame_to_ruletime(scenario.maxSteps, scenario.timestep)
solver = Solver(max_ruletime)
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
