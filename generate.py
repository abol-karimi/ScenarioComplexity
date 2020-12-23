import intersection_monitor
import scenic
from solver import Solver

params = {'map': './maps/Town05.xodr',
          'carla_map': 'Town05'}
scenario = scenic.scenarioFromFile('ego.scenic', params=params)
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
sim_result = simulator.simulate(scene, maxSteps=100)
blueprints = scene.params['blueprints']

params = {'map': './maps/Town05.xodr',
          'carla_map': 'Town05',
          'sim_result': sim_result,
          'blueprints': blueprints}
scenario = scenic.scenarioFromFile('nonego.scenic', params=params)
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
sim_result = simulator.simulate(scene, maxSteps=100)
blueprints = scene.params['blueprints']

params = {'map': './maps/Town05.xodr',
          'carla_map': 'Town05',
          'sim_result': sim_result,
          'blueprints': blueprints}
scenario = scenic.scenarioFromFile('ego.scenic', params=params)
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
sim_result = simulator.simulate(scene, maxSteps=100)
# for atom in intersection_monitor.monitor.geometry:
#     print(atom)
# for event in intersection_monitor.monitor.events:
#     print(event)

# solver = Solver("uncontrolled-4way.lp")
# solver.add_atoms(intersection_monitor.monitor.geometry)
# solver.add_atoms(intersection_monitor.monitor.events)
# solver.solve()
