import pickle
import scenic

with open('root0.sc', 'rb') as inFile:
    scenario = pickle.load(inFile)

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_id': scenario.intersection_id,
          'maneuver_id': scenario.maneuver_id,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': False}

print('Play the loaded scenario...')
params['trajectory'] = scenario.trajectory
params['blueprints'] = scenario.blueprints
params['vehicleLightStates'] = scenario.vehicleLightStates
scenic_scenario = scenic.scenarioFromFile(
    'replay.scenic', params=params)
scene, iterations = scenic_scenario.generate()
simulator = scenic_scenario.getSimulator()
sim_result_replay = simulator.simulate(scene, maxSteps=scenario.maxSteps)
