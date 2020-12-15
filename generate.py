import intersection_monitor
import scenic

scenario = scenic.scenarioFromFile('events.scenic')
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
intersection_monitor.monitor.set_world(simulator.world)
simulator.simulate(scene, maxSteps=200)
