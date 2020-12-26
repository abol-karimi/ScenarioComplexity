from intersection_monitor import monitor
import scenic

maxSteps = 300
render = False
blueprints = {'ego': 'vehicle.tesla.model3'}
sim_result = None

for i in range(1):
    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints}
    scenario = scenic.scenarioFromFile('ego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    simulator.render = render
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']

    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints}
    scenario = scenic.scenarioFromFile('nonego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    simulator.render = render
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']

    monitor.nonego_solution()
