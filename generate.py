from intersection_monitor import monitor
import scenic

maxSteps = 500
timestep = 0.05
weather = 'ClearSunset'
render = False
monitor.max_realtime = maxSteps*timestep
monitor.timestep = timestep

params = {'map': './maps/Town05.xodr',
          'carla_map': 'Town05',
          'timestep': timestep,
          'weather': weather,
          'render': render}
sim_result = None
blueprints = {'ego': 'vehicle.tesla.model3'}
vehicleLightStates = {}

for i in range(2):
    print('Finding a solution for the ego...')
    params['sim_result'] = sim_result
    params['blueprints'] = blueprints
    params['vehicleLightStates'] = vehicleLightStates
    scenario = scenic.scenarioFromFile('ego.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_ego = simulator.simulate(scene, maxSteps=maxSteps)

    print('Finding a new nonego whose right is violated...')
    params['sim_result'] = sim_result_ego
    params['blueprints'] = scene.params['blueprints']
    params['vehicleLightStates'] = scene.params['vehicleLightStates']
    scenario = scenic.scenarioFromFile(
        'nonego_trajectory.scenic', params=params)
    scene, iterations = scenario.generate()
    simulator = scenario.getSimulator()
    sim_result_nonego = simulator.simulate(scene, maxSteps=maxSteps)
    monitor.nonego_solution(sim_result_nonego)

    print('Play the solution for the nonego...')
    for i in range(maxSteps+1):
        sim_result_nonego.trajectory[i]['ego'] = sim_result_ego.trajectory[i]['ego']
    params['sim_result'] = sim_result_nonego
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
