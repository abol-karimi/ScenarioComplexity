from scenario import Scenario
from generator import Generator
import scenic

scenario = Scenario()

params = {'map': scenario.map_path,
          'carla_map': scenario.map_name,
          'intersection_id': scenario.intersection_id,
          'maneuver_id': scenario.maneuver_id,
          'timestep': scenario.timestep,
          'weather': scenario.weather,
          'render': False}

generator = Generator(map_path=scenario.map_path,
                      intersection_id=scenario.intersection_id,
                      timestep=scenario.timestep,
                      maxSteps=scenario.maxSteps)
for i in range(3):
    scenario = generator.extend(scenario)

    print('Play the extended scenario...')
    params['trajectory'] = scenario.trajectory
    params['blueprints'] = scenario.blueprints
    params['vehicleLightStates'] = scenario.vehicleLightStates
    scenic_scenario = scenic.scenarioFromFile(
        'replay.scenic', params=params)
    scene, iterations = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    sim_result_replay = simulator.simulate(scene, maxSteps=scenario.maxSteps)
