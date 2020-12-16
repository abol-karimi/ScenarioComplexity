import intersection_monitor
import scenic

scenario = scenic.scenarioFromFile('events.scenic')
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
simulator.simulate(scene, maxSteps=200)

for event in intersection_monitor.monitor.events:
    print(event)
