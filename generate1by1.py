from generator import Generator
import scenic

import intersection_monitor

maxSteps = 700
timestep = 0.05
weather = 'ClearSunset'
render = False
map_path = './maps/Town05.xodr'
map_name = 'Town05'
intersection_id = 3  # unsignalized four-way intersection in Town05

monitor = intersection_monitor.Monitor()
monitor.timestep = timestep
monitor.maxSteps = maxSteps

params = {'map': map_path,
          'carla_map': map_name,
          'intersection_id': intersection_id,
          'timestep': timestep,
          'weather': weather,
          'render': render,
          'event_monitor': monitor}
sim_result = None
blueprints = {'ego': 'vehicle.tesla.model3'}
vehicleLightStates = {}

generator = Generator(map_path=map_path,
                      intersection_id=intersection_id,
                      timestep=timestep,
                      maxSteps=maxSteps)
nonegos = set()

for i in range(2):
    monitor.events['ego'] = []
    print('Sampling an ego trajectory...')
    params['sim_result'] = sim_result
    params['blueprints'] = blueprints
    params['vehicleLightStates'] = vehicleLightStates
    scenario = scenic.scenarioFromFile('ego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_ego = simulator.simulate(scene, maxSteps=maxSteps)

    print('Finding a solution for the ego...')
    generator.events = monitor.events
    generator.ego_solution(sim_result_ego)

    print('Play the solution for the ego...')
    if sim_result:
        for i in range(maxSteps+1):
            sim_result.trajectory[i]['ego'] = sim_result_ego.trajectory[i]['ego']
    else:
        sim_result = sim_result_ego
    params['sim_result'] = sim_result
    params['blueprints'] = scene.params['blueprints']
    params['vehicleLightStates'] = scene.params['vehicleLightStates']
    scenario = scenic.scenarioFromFile(
        'replay.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_replay = simulator.simulate(scene, maxSteps=maxSteps)

    print('Sampleing a new nonego trajectory...')
    nonego = f'car{len(nonegos)}'
    nonegos.add(nonego)

    params['carName'] = nonego
    params['sim_result'] = sim_result_replay
    params['blueprints'] = scene.params['blueprints']
    params['vehicleLightStates'] = scene.params['vehicleLightStates']
    scenario = scenic.scenarioFromFile('nonego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_nonego = simulator.simulate(scene, maxSteps=maxSteps)

    print('Finding a new nonego whose right is violated by ego...')
    generator.events = monitor.events
    generator.nonego = nonego
    generator.nonego_solution(sim_result_nonego)

    print('Play the solution for the nonego...')
    for i in range(maxSteps+1):
        sim_result_replay.trajectory[i][nonego] = sim_result_nonego.trajectory[i][nonego]
    params['sim_result'] = sim_result_replay
    params['blueprints'] = scene.params['blueprints']
    params['vehicleLightStates'] = scene.params['vehicleLightStates']
    scenario = scenic.scenarioFromFile(
        'replay.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_replay = simulator.simulate(scene, maxSteps=maxSteps)

    # Set up the parameters for the next iteration
    sim_result = sim_result_replay
    blueprints = scene.params['blueprints']
    vehicleLightStates = scene.params['vehicleLightStates']
