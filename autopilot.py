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

params = {'map': scenario.map_path,  # scenic.simulators.carla.model
          'carla_map': scenario.map_name,  # scenic.simulators.carla.model
          'timestep': scenario.timestep,  # scenic.simulators.carla.model
          'weather': scenario.weather,  # scenic.simulators.carla.model
          'render': True,  # scenic.simulators.carla.model
          'replay_scenario': scenario,
          'event_monitor': monitor,
          'stop_speed_threshold': 0.01,  # meters/seconds
          'aggressiveness': args.aggressiveness}

print('Play an autopilot ego in the scenario...')
scenic_scenario = scenic.scenarioFromFile(
    'autopilot.scenic', params=params)
scene, _ = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
simulator.simulate(scene, maxSteps=scenario.maxSteps)

atoms = []
network = Network.fromFile(scenario.map_path)
atoms += geometry_atoms(network, scenario.intersection_uid)

event_atoms = []
for event in monitor.events['ego']:
    ruletime = frame_to_ruletime(event.frame, scenario.timestep)
    atom = event.withTime(ruletime)
    event_atoms.append(atom)

for car in scenario.events.keys():
    if car in {'ego', 'illegal'}:
        continue
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
