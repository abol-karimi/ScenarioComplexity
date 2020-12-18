import intersection_monitor
import scenic
from solver import Solver

scenario = scenic.scenarioFromFile('events.scenic')
scene, iterations = scenario.generate()
simulator = scenario.getSimulator()
simulator.simulate(scene, maxSteps=200)

for atom in intersection_monitor.monitor.geometry:
    print(atom)
for event in intersection_monitor.monitor.events:
    print(event)

solver = Solver("uncontrolled-4way.lp")
solver.add_atoms(intersection_monitor.monitor.geometry)
solver.add_atoms(intersection_monitor.monitor.events)
solver.solve()
