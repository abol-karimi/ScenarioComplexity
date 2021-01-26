#!/home/ak/Scenic/.venv/bin/python
import sys
import getopt
from generator import load_geometry, frame_to_ruletime


def main(argv):
    inputfile = ''
    try:
        opts, _ = getopt.getopt(argv, "hi:", ["ifile="])
    except getopt.GetoptError:
        print('extend.py -i <inputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('extend.py -i <inputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg

    import pickle
    import scenic

    with open(inputfile, 'rb') as inFile:
        scenario = pickle.load(inFile)

    import intersection_monitor
    monitor = intersection_monitor.Monitor()

    params = {'map': scenario.map_path,
              'carla_map': scenario.map_name,
              'intersection_id': scenario.intersection_id,
              'maneuver_id': scenario.maneuver_id,
              'timestep': scenario.timestep,
              'weather': scenario.weather,
              'render': True,
              'event_monitor': monitor}

    print('Play an autopilot ego in the scenario...')
    params['trajectory'] = scenario.trajectory
    params['blueprints'] = scenario.blueprints
    scenic_scenario = scenic.scenarioFromFile(
        'autopilot.scenic', params=params)
    scene, _ = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    simulator.simulate(scene, maxSteps=scenario.maxSteps)

    atoms = []
    atoms += load_geometry(scenario.map_path, scenario.intersection_id)

    for event in monitor.events['ego']:
        ruletime = frame_to_ruletime(event.frame, scenario.timestep)
        atom = event.withTime(ruletime)
        atoms.append(atom)

    for car in scenario.events.keys():
        if not car in {'ego', 'illegal'}:
            atoms += [event.withTime(frame_to_ruletime(event.frame, scenario.timestep))
                      for event in scenario.events[car]]

    from solver import Solver
    max_ruletime = frame_to_ruletime(scenario.maxSteps, scenario.timestep)
    solver = Solver(max_ruletime)
    solver.load(scenario.rules_path)
    solver.add_atoms(atoms)

    model = solver.solve()

    sol_names = {'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                 'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
    print("Logical solution: ")
    for atom in model:
        if atom.name in sol_names:
            print(f'\t{atom}')


if __name__ == "__main__":
    main(sys.argv[1:])
