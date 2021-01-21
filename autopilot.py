#!/home/ak/Scenic/.venv/bin/python
import sys
import getopt


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

    print('Play the loaded scenario...')
    params['trajectory'] = scenario.trajectory
    params['blueprints'] = scenario.blueprints
    params['vehicleLightStates'] = scenario.vehicleLightStates
    scenic_scenario = scenic.scenarioFromFile(
        'autopilot.scenic', params=params)
    scene, _ = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    simulator.simulate(scene, maxSteps=scenario.maxSteps)

    atoms = []
    atoms += load_geometry(scenario.map_path, scenario.intersection_id)

    atoms += [event.withTime(frame_to_ruletime(event.frame, scenario.timestep))
              for event in monitor.events['ego']]
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
        args = atom.arguments
        if (atom.name in sol_names) and str(args[0]) == 'ego':
            print(f'\t{atom}')


def load_geometry(map_path, intersection_id):
    from signals import SignalType
    from scenic.domains.driving.roads import Network
    network = Network.fromFile(map_path)
    intersection = network.intersections[intersection_id]
    geometry = []
    for maneuver in intersection.maneuvers:
        lane = maneuver.connectingLane
        fork = maneuver.startLane
        exit = maneuver.endLane
        geometry.append(
            f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')
        signal = SignalType.from_maneuver(maneuver).name.lower()
        geometry.append(
            f'laneCorrectSignal({lane.uid}, {signal})')

    for maneuver in intersection.maneuvers:
        for conflict in maneuver.conflictingManeuvers:
            geometry.append(
                f'overlaps({maneuver.connectingLane.uid}, {conflict.connectingLane.uid})')

    roads = intersection.roads
    incomings = intersection.incomingLanes
    road2incomings = {road.uid: [] for road in roads}
    for incoming in incomings:
        road2incomings[incoming.road.uid].append(incoming.uid)
    # An intersection stores the intersecting roads in CW or CCW order.
    # Assuming the order is CCW, then:
    for i in range(len(roads)):
        j = (i+1) % len(roads)
        lefts = road2incomings[roads[i].uid]
        rights = road2incomings[roads[j].uid]
        geometry += [
            f'isOnRightOf({right}, {left})' for left in lefts for right in rights]
    return geometry


def realtime_to_ruletime(t):
    return int(t*2)


def frame_to_realtime(frame, timestep):
    return frame*timestep


def frame_to_ruletime(frame, timestep):
    realtime = frame_to_realtime(frame, timestep)
    return realtime_to_ruletime(realtime)


if __name__ == "__main__":
    main(sys.argv[1:])
