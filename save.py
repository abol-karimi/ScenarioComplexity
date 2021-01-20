import pickle
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

scenario = generator.extend(scenario)

with open('root0.sc', 'wb') as outFile:
    pickle.dump(scenario, outFile)
