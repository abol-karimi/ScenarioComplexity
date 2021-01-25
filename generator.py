def realtime_to_ruletime(t):
    return int(t*2)


def ruletime_to_realtime(T):
    return T/2


def realtime_to_frame(t, timestep):
    return int(t/timestep)


def frame_to_realtime(frame, timestep):
    return frame*timestep


def ruletime_to_frame(T, timestep):
    realtime = ruletime_to_realtime(T)
    return realtime_to_frame(realtime, timestep)


def frame_to_ruletime(frame, timestep):
    realtime = frame_to_realtime(frame, timestep)
    return realtime_to_ruletime(realtime)


def frame_to_distance(sim, car):
    trajectory = sim.trajectory
    frame2distance = [0]*len(trajectory)

    for i in range(len(trajectory)-1):
        pi = trajectory[i][car][0]
        pii = trajectory[i+1][car][0]
        frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

    return frame2distance


def load_geometry(map_path, intersection_id):
    from signals import SignalType
    from scenic.domains.driving.roads import Network
    network = Network.fromFile(map_path)
    intersection = network.intersections[intersection_id]
    maneuvers = intersection.maneuvers
    geometry = []
    for maneuver in maneuvers:
        lane = maneuver.connectingLane
        fork = maneuver.startLane
        exit = maneuver.endLane
        geometry.append(
            f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')
        signal = SignalType.from_maneuver(maneuver).name.lower()
        geometry.append(
            f'laneCorrectSignal({lane.uid}, {signal})')

    for i in range(len(maneuvers)):
        li = maneuvers[i].connectingLane
        for j in range(i+1, len(maneuvers)):
            lj = maneuvers[j].connectingLane
            if li.intersects(lj):
                geometry.append(f'overlaps({li.uid}, {lj.uid})')
                geometry.append(f'overlaps({lj.uid}, {li.uid})')

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


def traj_constraints(scenario, events, frame2distance, maxSpeed):
    # Index car's events by their frame
    frame2events = {event.frame: [] for event in events}
    for event in events:
        frame2events[event.frame].append(event)

    # Constrainst atoms
    atoms = []

    # Car's simultaneous events
    for frame in frame2events.keys():
        frame_events = frame2events[frame]
        for i in range(len(frame_events)-1):
            atoms.append(
                f':- {frame_events[i].withTime("T1")}, {frame_events[i+1].withTime("T2")}, T1 != T2')

    # Car's non-simultaneous events
    # Two non-simultaneous events may have the same ruletime (logical time)
    frames = sorted(frame2events.keys())
    for i in range(len(frames)-1):
        ei = frame2events[frames[i]][0]
        eii = frame2events[frames[i+1]][0]
        atoms += [f':- {ei.withTime("T1")}, {eii.withTime("T2")}, T1 > T2']

    # Car's speed is bounded between any two events
    for i in range(len(frames)):
        ei = frame2events[frames[i]][0]
        di = frame2distance[frames[i]]
        # Average speed from begining to ti
        atoms += [
            f':- {ei.withTime("T")}, {int(2*di/maxSpeed)+1} > T']
        dinf = frame2distance[-1]
        tinf = frame_to_ruletime(scenario.maxSteps, scenario.timestep)
        # Average speed from ti to the end
        atoms += [
            f':- {ei.withTime("T")}, {int(2*(dinf-di)/maxSpeed)+1} > {tinf} - T']
        # Average speed from ti to later events
        for j in range(i+1, len(frames)):
            ej = frame2events[frames[j]][0]
            dj = frame2distance[frames[j]]
            delta = int(2*(dj-di)/maxSpeed)+1
            # 0 < delta <= tj - ti
            atoms += [
                f':- {ei.withTime("T1")}, {ej.withTime("T2")}, {delta} > T2 - T1']

    # Generate car events
    for event in events:
        atoms += [f'{{ {event.withTime("T")} : time(T) }} = 1']

    return atoms


def nocollision(car1, car2):
    atoms = []
    # They don't enter the overlap at the same time
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car1}, L2, T), leftLaneAtTime({car2}, L1, T)']
    # If car2 enters the overlap first, it exits it before car1 enters it.
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car1}, L2, T),'
              f'enteredLaneByTime({car2}, L1, T), not leftLaneByTime({car2}, L1, T)']
    # If car1 enters the overlap first, it exits it before car2 enters it.
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car2}, L1, T),'
              f'enteredLaneByTime({car1}, L2, T), not leftLaneByTime({car1}, L2, T)']

    return atoms


def model_to_events(model, events_all, car):
    event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                   'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

    # To connect logic solution with events
    timeless2event = {event.withTime(
        ''): event for event in events_all[car]}

    # A mapping from new ruletimes to old events
    ruletime2events = {}
    for atom in model:
        name = atom.name
        args = atom.arguments
        if not (str(args[0]) == car and name in event_names):
            continue
        if len(args) == 3:
            timeless = f'{name}({args[0]}, {args[1]}, )'
        else:
            timeless = f'{name}({args[0]}, {args[1]}, {args[2]}, )'
        ruletime = int(str(args[-1]))
        if not ruletime in ruletime2events:
            ruletime2events[ruletime] = [timeless2event[timeless]]
        else:
            ruletime2events[ruletime] += [timeless2event[timeless]]

    return ruletime2events


def logical_solution(scenario, events_all, nonego, frame2distance_ego, frame2distance_illegal, frame2distance_nonego, maxSpeed):
    atoms = load_geometry(scenario.map_path, scenario.intersection_id)

    old_nonegos = {car for car in scenario.events.keys() if not car in {
        'ego', 'illegal'}}
    for car in old_nonegos:
        atoms += [event.withTime(frame_to_ruletime(event.frame, scenario.timestep))
                  for event in scenario.events[car]]

    atoms += traj_constraints(scenario,
                              events_all['ego'], frame2distance_ego, maxSpeed)
    atoms += traj_constraints(scenario,
                              events_all['illegal'], frame2distance_illegal, maxSpeed)
    atoms += traj_constraints(scenario,
                              events_all[nonego], frame2distance_nonego, maxSpeed)

    # No collision
    atoms += nocollision('ego', nonego)
    for car in old_nonegos:
        atoms += nocollision(car, 'ego')
        atoms += nocollision(car, nonego)

    # Enforce ego's legal behavior
    atoms += [f':- violatesRightOf(ego, _)']

    # Enforce nonego's legal behavior
    atoms += [f':- V != illegal, violatesRightOf({nonego}, V)']

    # Evidence that new scenario is strictly harder
    atoms += [f':- not violatesRightOf(illegal, {nonego})']

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

    ruletime2events_ego = model_to_events(model, events_all, 'ego')
    ruletime2events_nonego = model_to_events(model, events_all, nonego)

    return ruletime2events_ego, ruletime2events_nonego


def events_to_trajectory(scenario, ruletime2events, car, trajectory, frame2distance, ruletime2distances):
    # Interpolate car's trajectory based on
    #  its new (frame, distance) points:
    p_f = [0]
    p_d = [0]
    ruletimes = sorted(ruletime2distances.keys())  # for np.interp()
    for ruletime in ruletimes:
        ds = ruletime2distances[ruletime]
        for d in ds:
            fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
            frame = ruletime_to_frame(
                ruletime + fraction, scenario.timestep)
            p_f += [frame]
            p_d += [d]
    p_f += [len(frame2distance)-1]
    p_d += [frame2distance[-1]]

    # Linearly interpolate the (frame, distance) points
    import numpy as np
    new2distance = np.interp(range(len(trajectory)), p_f, p_d)

    import matplotlib.pyplot as plt
    plt.plot(frame2distance, 'g')
    plt.plot(p_f, p_d, 'ro')
    plt.plot(new2distance)

    plt.show()

    # The new trajectory
    new2old = []
    old = 0
    for new in range(len(trajectory)):
        while old < len(trajectory)-1 and frame2distance[old] < new2distance[new]:
            old += 1
        new2old += [old]

    new_traj = []
    for frame in range(len(trajectory)):
        new_traj += [trajectory[new2old[frame]][car]]

    return new_traj


def solution(scenario, events_all, nonego, sim_ego, sim_nonego, maxSpeed):
    import copy
    events_all['illegal'] = []
    for event in events_all['ego']:
        event_ill = copy.copy(event)
        event_ill.vehicle = 'illegal'
        events_all['illegal'] += [event_ill]

    frame2distance_ego = frame_to_distance(sim_ego, 'ego')
    frame2distance_illegal = frame_to_distance(
        sim_ego, 'ego')  # TODO can use frame2distance_ego
    frame2distance_nonego = frame_to_distance(sim_nonego, nonego)

    # TODO Concretize the illegal trajectory as well
    ruletime2events_ego, ruletime2events_nonego = logical_solution(scenario, events_all, nonego, frame2distance_ego,
                                                                   frame2distance_illegal, frame2distance_nonego, maxSpeed)

    # Distances of events of a ruletime in increasing order
    ruletime2distances_ego = {}
    for ruletime, events in ruletime2events_ego.items():
        distances = [frame2distance_ego[event.frame]
                     for event in events]
        distances_sorted = sorted(set(distances))
        ruletime2distances_ego[ruletime] = distances_sorted

    # Distances of events of a ruletime in increasing order
    ruletime2distances_nonego = {}
    for ruletime, events in ruletime2events_nonego.items():
        distances = [frame2distance_nonego[event.frame]
                     for event in events]
        distances_sorted = sorted(set(distances))
        ruletime2distances_nonego[ruletime] = distances_sorted

    trajectory_ego = sim_ego.trajectory
    trajectory_nonego = sim_nonego.trajectory

    # Interpolate the events to a new trajectory
    new_traj_ego = events_to_trajectory(scenario,
                                        ruletime2events_ego, 'ego', trajectory_ego, frame2distance_ego, ruletime2distances_ego)
    new_traj_nonego = events_to_trajectory(scenario,
                                           ruletime2events_nonego, nonego, trajectory_nonego, frame2distance_nonego, ruletime2distances_nonego)

    traj_prev = scenario.trajectory
    # When extending an empty scenario
    if not traj_prev:
        traj_prev = [{} for i in range(len(trajectory_ego))]

    # Trajectories of the new vehicles
    for frame in range(len(traj_prev)):
        traj_prev[frame]['ego'] = new_traj_ego[frame]
        traj_prev[frame][nonego] = new_traj_nonego[frame]

    # Update timing of new cars' events
    for ruletime, events in ruletime2events_ego.items():
        ds = ruletime2distances_ego[ruletime]
        for event in events:
            d = frame2distance_ego[event.frame]
            fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
            frame = ruletime_to_frame(
                ruletime + fraction, scenario.timestep)
            event.frame = frame
    for ruletime, events in ruletime2events_nonego.items():
        ds = ruletime2distances_nonego[ruletime]
        for event in events:
            d = frame2distance_nonego[event.frame]
            fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
            frame = ruletime_to_frame(
                ruletime + fraction, scenario.timestep)
            event.frame = frame

    return traj_prev


def extend(scenario, nonego_maneuver_id=0, nonego_spawn_distance=10, nonego_blueprint='vehicle.tesla.model3', maxSpeed=7):
    import intersection_monitor
    monitor = intersection_monitor.Monitor()

    import scenic
    render = False

    params = {'map': scenario.map_path,
              'carla_map': scenario.map_name,
              'intersection_id': scenario.intersection_id,
              'timestep': scenario.timestep,
              'weather': scenario.weather,
              'render': render,
              'event_monitor': monitor}

    monitor.events['ego'] = []
    print('Sample an ego trajectory...')
    params['car_name'] = 'ego'
    params['maneuver_id'] = scenario.maneuver_id['ego']
    params['spawn_distance'] = 20
    params['car_blueprint'] = scenario.blueprints['ego']
    scenic_scenario = scenic.scenarioFromFile(
        'trajectory.scenic', params=params)
    scene, _ = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    settings = simulator.world.get_settings()
    settings.no_rendering_mode = True
    simulator.world.apply_settings(settings)
    sim_result_ego = simulator.simulate(scene, maxSteps=scenario.maxSteps)

    print('Sample a nonego trajectory...')
    nonego = f'car{len(scenario.blueprints)}'
    params['car_name'] = nonego
    params['maneuver_id'] = nonego_maneuver_id
    params['spawn_distance'] = nonego_spawn_distance
    params['car_blueprint'] = nonego_blueprint
    scenic_scenario = scenic.scenarioFromFile(
        'trajectory.scenic', params=params)
    scene, _ = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    settings = simulator.world.get_settings()
    settings.no_rendering_mode = True
    simulator.world.apply_settings(settings)
    sim_result_nonego = simulator.simulate(
        scene, maxSteps=scenario.maxSteps)

    # Find a strict extension of the given scenario
    events = {car: event for car, event in scenario.events.items()}
    events.update(monitor.events)
    trajectory = solution(
        scenario, events, nonego, sim_result_ego, sim_result_nonego, maxSpeed)

    from scenario import Scenario
    scenario_ext = Scenario()
    scenario_ext.maxSteps = scenario.maxSteps
    scenario_ext.timestep = scenario.timestep
    scenario_ext.weather = scenario.weather
    scenario_ext.map_path = scenario.map_path
    scenario_ext.map_name = scenario.map_name
    scenario_ext.intersection_id = scenario.intersection_id
    scenario_ext.rules_path = scenario.rules_path
    scenario_ext.blueprints = {nonego: nonego_blueprint}
    scenario_ext.blueprints.update(scenario.blueprints)
    scenario_ext.maneuver_id = {nonego: nonego_maneuver_id}
    scenario_ext.maneuver_id.update(scenario.maneuver_id)
    scenario_ext.trajectory = trajectory
    scenario_ext.events = events

    return scenario_ext
