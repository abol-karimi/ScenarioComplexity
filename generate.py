from intersection_monitor import monitor
import scenic

maxSteps = 600
timestep = 0.05
weather = 'ClearSunset'
render = False
blueprints = {'ego': 'vehicle.tesla.model3'}
sim_result = None

for i in range(1):
    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints,
              'timestep': timestep,
              'weather': weather,
              'render': render}
    scenario = scenic.scenarioFromFile('ego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']

    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints,
              'timestep': timestep,
              'weather': weather,
              'render': render}
    scenario = scenic.scenarioFromFile('nonego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']

    monitor.nonego_solution()
