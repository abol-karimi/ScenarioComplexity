from intersection_monitor import monitor
import scenic

maxSteps = 500
timestep = 0.05
weather = 'ClearSunset'
render = False
blueprints = {'ego': 'vehicle.tesla.model3'}
vehicleLightStates = {}
sim_result = None
monitor.max_realtime = maxSteps*timestep
monitor.timestep = timestep

for i in range(2):
    print('Finding a solution for the ego...')
    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints,
              'vehicleLightStates': vehicleLightStates,
              'timestep': timestep,
              'weather': weather,
              'render': render}
    scenario = scenic.scenarioFromFile('ego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']
    vehicleLightStates = scene.params['vehicleLightStates']

    print('Finding a new nonego whose right is violated...')
    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints,
              'vehicleLightStates': vehicleLightStates,
              'timestep': timestep,
              'weather': weather,
              'render': render}
    scenario = scenic.scenarioFromFile(
        'nonego_trajectory.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
    blueprints = scene.params['blueprints']
    vehicleLightStates = scene.params['vehicleLightStates']

    monitor.nonego_solution(sim_result)

    print('Play the solution for the nonego...')
    params = {'map': './maps/Town05.xodr',
              'carla_map': 'Town05',
              'sim_result': sim_result,
              'blueprints': blueprints,
              'vehicleLightStates': vehicleLightStates,
              'timestep': timestep,
              'weather': weather,
              'render': render}
    scenario = scenic.scenarioFromFile(
        'replay.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result = simulator.simulate(scene, maxSteps=maxSteps)
