import intersection_monitor
import scenic

scenario = scenic.scenarioFromFile('events.scenic')
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
simulator.simulate(scene, maxSteps=200)
