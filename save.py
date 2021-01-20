import pickle
from scenario import Scenario
from generator import Generator
import scenic

scenario = Scenario()

generator = Generator(map_path=scenario.map_path,
                      intersection_id=scenario.intersection_id,
                      timestep=scenario.timestep,
                      maxSteps=scenario.maxSteps)

scenario = generator.extend(scenario)

with open('root0.sc', 'wb') as outFile:
    pickle.dump(scenario, outFile)
