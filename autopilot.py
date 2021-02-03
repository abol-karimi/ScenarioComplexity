#!/home/ak/Scenic/.venv/bin/python
from scenic.domains.driving.roads import Network
from solver import Solver
import intersection_monitor
import scenic
import pickle
import argparse
from generator import geometry_atoms, frame_to_ruletime


parser = argparse.ArgumentParser(
    description='play the given scenario with a Carla autopilot driving the ego.')
parser.add_argument('inputfile', help='filename of the given scenario')
parser.add_argument('-a', '--aggressiveness', choices=['cautious', 'normal', 'aggressive'],
                    default='normal', help='aggressiveness of Carla BehaviorAgent')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

monitor = intersection_monitor.Monitor()

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_uid': scenario.intersection_uid,
          'maneuver_uid': scenario.maneuver_uid,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': True,
          'event_monitor': monitor}

print('Play an autopilot ego in the scenario...')
params['trajectory'] = scenario.trajectory
params['blueprints'] = scenario.blueprints
params['aggressiveness'] = args.aggressiveness
scenic_scenario = scenic.scenarioFromFile(
    'autopilot.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)

atoms = []
network = Network.fromFile(scenario.map_path)
atoms += geometry_atoms(network, scenario.intersection_uid)

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
