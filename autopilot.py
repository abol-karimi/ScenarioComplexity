#!/home/ak/Scenic/.venv/bin/python
from solver import Solver
import intersection_monitor
import scenic
import pickle
import argparse
from generator import load_geometry, frame_to_ruletime


parser = argparse.ArgumentParser(
    description='play the given scenario with a Carla autopilot driving the ego.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

monitor = intersection_monitor.Monitor()

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_id': scenario.intersection_id,
          'maneuver_id': scenario.maneuver_id,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': True,
          'event_monitor': monitor}

print('Play an autopilot ego in the scenario...')
params['trajectory'] = scenario.trajectory
params['blueprints'] = scenario.blueprints
scenic_scenario = scenic.scenarioFromFile(
    'autopilot.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)

atoms = []
atoms += load_geometry(scenario.map_path, scenario.intersection_id)

for event in monitor.events['ego']:
    ruletime = frame_to_ruletime(event.frame, scenario.timestep)
    atom = event.withTime(ruletime)
    atoms.append(atom)

for car in scenario.events.keys():
    if not car in {'ego', 'illegal'}:
        atoms += [event.withTime(frame_to_ruletime(event.frame, scenario.timestep))
                  for event in scenario.events[car]]

max_ruletime = frame_to_ruletime(scenario.maxSteps, scenario.timestep)
solver = Solver(max_ruletime)
solver.load(scenario.rules_path)
solver.add_atoms(atoms)

model = solver.solve()

sol_names = {'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
             'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
print("Logical solution: ")
for atom in model:
    if atom.name in sol_names:
        print(f'\t{atom}')
