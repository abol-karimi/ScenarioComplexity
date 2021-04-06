import math


# Rounds r >=0 down to precision number of decimal places.
def round_up(r, precision=3):
    coeff = 10**precision
    return math.ceil(r*coeff)/coeff


# Rounds r >=0 down to precision number of decimal places.
def round_down(r, precision=3):
    coeff = 10**precision
    return math.floor(r*coeff)/coeff


# Returns sign(r)*round_up(abs(r), precision)
def round_norm_up(r, precision=3):
    if r >= 0:
        return round_up(r, precision)
    else:
        return -round_up(-r, precision)


# Returns sign(r)*round_down(abs(r), precision)
def round_norm_down(r, precision=3):
    if r >= 0:
        return round_down(r, precision)
    else:
        return -round_down(-r, precision)


def realtime_to_ruletime(t):
    return int(t*2)


def ruletime_to_realtime(T):
    return T/2


def realtime_to_frame(t, timestep):
    return int(realtime_to_frame_float(t, timestep))


def realtime_to_frame_float(t, timestep):
    return t/timestep


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


def geometry_atoms(network, intersection_uid):
    """Assumes the correct map is loaded in CARLA server."""
    from signals import SignalType
    intersection = network.elements[intersection_uid]
    maneuvers = intersection.maneuvers
    geometry = []
    for maneuver in maneuvers:
        lane = maneuver.connectingLane
        fork = maneuver.startLane
        exit = maneuver.endLane
        geometry.append(
            f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')

    for maneuver in maneuvers:
        lane = maneuver.connectingLane
        signal = SignalType.from_maneuver(maneuver).name.lower()
        geometry.append(
            f'laneCorrectSignal({lane.uid}, {signal})')

    for i in range(len(maneuvers)):
        li = maneuvers[i].connectingLane
        geometry.append(f'overlaps({li.uid}, {li.uid})')
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
    import math
    for i in range(len(roads)):
        j = (i+1) % len(roads)
        lefts = road2incomings[roads[i].uid]
        rights = road2incomings[roads[j].uid]
        l0 = network.elements[lefts[0]]
        r0 = network.elements[rights[0]]
        hl = l0.centerline[-1] - l0.centerline[-2]
        hr = r0.centerline[-1] - r0.centerline[-2]
        if abs(math.pi - abs(hr.angleWith(hl))) < math.pi/6:
            continue
        geometry += [
            f'isOnRightOf({right}, {left})' for left in lefts for right in rights]

    # To detect stop signs
    import carla
    client = carla.Client('127.0.0.1', 2000)
    world = client.get_world()
    map = world.get_map()

    from scenic.simulators.carla.utils.utils import scenicToCarlaLocation
    for lane in incomings:
        end = lane.centerline[-1]
        point = lane.flowFrom(end, -6.0)
        loc = scenicToCarlaLocation(point, world=world)
        wp = map.get_waypoint(loc)
        landmarks = wp.get_landmarks_of_type(
            6.0, '206')  # 206: stop sign or utencil
        if len(landmarks) > 0:
            geometry += [f'hasStopSign({lane.uid})']

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


def nocollision_other(car1, car2):
    atoms = []
    # They don't enter the overlap at the same time
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car1}, L2, T), enteredLaneAtTime({car2}, L1, T)']
    # If car2 enters the overlap first, it exits it before car1 enters it.
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car2}, L1, T1),'
              f'enteredLaneAtTime({car1}, L2, T2),'
              f'T1 < T2,'
              f'not leftLaneByTime({car2}, L1, T2)']
    # If car1 enters the overlap first, it exits it before car2 enters it.
    atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
              f'overlaps(L1, L2),'
              f'enteredLaneAtTime({car1}, L2, T1),'
              f'enteredLaneAtTime({car2}, L1, T2),'
              f'T1 < T2,'
              f'not leftLaneByTime({car1}, L2, T2)']

    return atoms


def nocollision_merge(car1, car2):
    return nocollision_other(car1, car2)


def nocollision_follow(follow, lead):
    atoms = []
    atoms += [f':- requestedLane({follow}, L),'
              f'enteredLaneAtTime({follow}, L1, T1),'
              f'L != L1,'
              f'not leftLaneByTime({lead}, L1, T1)']

    atoms += [f':- arrivedAtTime({follow}, T),'
              f'not enteredByTime({lead}, T)']

    atoms += [f':- exitedFromAtTime({lead}, E, T1),'
              f'exitedFromAtTime({follow}, E, T2),'
              f'T2 < T1 + 2']
    return atoms


def nocollision_fork(follow, lead):
    """Reduces the chance of collision but does not elliminate it."""
    atoms = []
    atoms += [f':- arrivedAtTime({follow}, T),'
              f'not enteredByTime({lead}, T)']
    atoms += [f':- requestedLane({lead}, L1),'
              f'requestedLane({follow}, L2),'
              f'L3 != L1, L3 != L2,'
              f'enteredLaneAtTime({lead}, L3, _),'
              f'enteredLaneAtTime({follow}, L3, T2),'
              f'not leftLaneByTime({lead}, L3, T2)']
    return atoms


def nocollision(network, scenario, nonego,
                nonego_maneuver_uid,
                nonego_spawn_distance,
                sim_ego, sim_nonego):
    atoms = []

    # Spawn distance for each car
    intersection = network.elements[scenario.intersection_uid]
    trajectory = scenario.trajectory
    spawn_distance = {}
    if trajectory:
        spawn_distance = {car: intersection.distanceTo(state[0])
                          for car, state in trajectory[0].items()}
    spawn_distance[nonego] = nonego_spawn_distance
    ego_loc0 = sim_ego.trajectory[0]['ego'][0]
    ego_spawn_distance = intersection.distanceTo(ego_loc0)
    spawn_distance['ego'] = ego_spawn_distance
    spawn_distance['illegal'] = ego_spawn_distance

    def follow_lead(car1, car2):
        if spawn_distance[car1] < spawn_distance[car2]:
            return car2, car1
        else:
            return car1, car2

    # No collision between ego and nonego
    nm = nonego_maneuver_uid
    em = scenario.maneuver_uid['ego']
    follow, lead = follow_lead('ego', nonego)
    if em[0] == nm[0] and em[2] == nm[2]:
        atoms += nocollision_follow(follow, lead)
    elif em[0] == nm[0] and em[2] != nm[2]:
        atoms += nocollision_fork(follow, lead)
    elif em[0] != nm[0] and em[2] == nm[2]:
        atoms += nocollision_merge('ego', nonego)
    else:
        atoms += nocollision_other('ego', nonego)

    # No collision between ego and old nonegos:
    old_nonegos = {car for car in scenario.events.keys()
                   if not car in {'ego', 'illegal'}}
    for old in old_nonegos:
        om = scenario.maneuver_uid[old]
        follow, lead = follow_lead('ego', old)
        if em[0] == om[0] and em[2] == om[2]:
            atoms += nocollision_follow(follow, lead)
        elif em[0] == om[0] and em[2] != om[2]:
            atoms += nocollision_fork(follow, lead)
        elif em[0] != om[0] and em[2] == om[2]:
            atoms += nocollision_merge('ego', old)
        else:
            atoms += nocollision_other('ego', old)

    # No collision between illegal and old nonegos:
    for old in old_nonegos:
        om = scenario.maneuver_uid[old]
        follow, lead = follow_lead('illegal', old)
        if em[0] == om[0] and em[2] == om[2]:
            atoms += nocollision_follow(follow, lead)
        elif em[0] == om[0] and em[2] != om[2]:
            atoms += nocollision_fork(follow, lead)
        elif em[0] != om[0] and em[2] == om[2]:
            atoms += nocollision_merge('illegal', old)
        else:
            atoms += nocollision_other('illegal', old)

    # No collision between nonego and old nonegos:
    for old in old_nonegos:
        om = scenario.maneuver_uid[old]
        follow, lead = follow_lead(nonego, old)
        if nm[0] == om[0] and nm[2] == om[2]:
            atoms += nocollision_follow(follow, lead)
        elif nm[0] == om[0] and nm[2] != om[2]:
            atoms += nocollision_fork(follow, lead)
        elif nm[0] != om[0] and nm[2] == om[2]:
            atoms += nocollision_merge(nonego, old)
        else:
            atoms += nocollision_other(nonego, old)

    return atoms


def model_to_events(model, events, frame2distance, car):
    """ Given an ASP model 'model', it extracts the new timing of the events of 'car' and
    returns a mapping from each new ruletime to corresponding events in 'events'.
    """

    event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                   'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

    # To connect logic solution with events
    timeless2event = {event.withTime(''): event
                      for event in events}

    # Tag events by their new logical time, and their distance along trajectory
    time_event_distance = []
    for atom in model:
        name = atom.name
        args = atom.arguments
        if not (str(args[0]) == car and name in event_names):
            continue
        if len(args) == 3:
            timeless = f'{name}({args[0]}, {args[1]}, )'
        else:
            timeless = f'{name}({args[0]}, {args[1]}, {args[2]}, )'
        logicalTime = int(str(args[-1]))
        event = timeless2event[timeless]
        distance = frame2distance[event.frame]
        time_event_distance += [(logicalTime, event, distance)]
    time_event_distance.sort(key=lambda triple: triple[1].frame)

    return time_event_distance


def logical_solution(scenario, sim_events,
                     nonego, nonego_maneuver_uid, nonego_spawn_distance,
                     sim_ego, sim_nonego,
                     frame2distance_ego,
                     frame2distance_illegal,
                     frame2distance_nonego,
                     maxSpeed,
                     extra_constraints):
    """ Given the events for ego, nonego, and illegal (in 'sim_events')
    and their distances along the corresponding car's trajectory (in 'frame2distance_*'),
    find a timing for the events that satisfies the logical constraints.
    """

    atoms = extra_constraints

    from scenic.domains.driving.roads import Network
    network = Network.fromFile(scenario.map_path)
    atoms += geometry_atoms(network, scenario.intersection_uid)

    old_nonegos = {car for car in scenario.events.keys() if not car in {
        'ego', 'illegal'}}
    for car in old_nonegos:
        atoms += [event.withTime(frame_to_ruletime(event.frame, scenario.timestep))
                  for event in scenario.events[car]]

    atoms += traj_constraints(scenario,
                              sim_events['ego'], frame2distance_ego, maxSpeed)
    atoms += traj_constraints(scenario,
                              sim_events['illegal'], frame2distance_illegal, maxSpeed)
    atoms += traj_constraints(scenario,
                              sim_events[nonego], frame2distance_nonego, maxSpeed)

    # No collision
    atoms += nocollision(network, scenario, nonego,
                         nonego_maneuver_uid,
                         nonego_spawn_distance,
                         sim_ego, sim_nonego)

    # Evidence that the new scenario has a solution
    atoms += [f':- V != illegal, violatesRightOf(ego, V)']
    atoms += [f':- violatesRule(ego, _)']

    # Evidence that the new scenario is strictly harder
    atoms += [f':- not violatesRightOf(illegal, {nonego})']
    atoms += [f':- V != {nonego}, V != ego, violatesRightOf(illegal, V)']
    atoms += [f':- violatesRule(illegal, _)']

    from solver import Solver
    max_ruletime = frame_to_ruletime(scenario.maxSteps, scenario.timestep)
    solver = Solver(max_ruletime)
    solver.load(scenario.rules_path)
    solver.add_atoms(atoms)

    model = solver.solve()

    sol_names = {'violatesRule', 'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                 'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
    print("Logical solution: ")
    for atom in model:
        if atom.name in sol_names:
            print(f'\t{atom}')

    time_event_distance_ego = model_to_events(
        model, sim_events['ego'], frame2distance_ego, 'ego')
    time_event_distance_nonego = model_to_events(
        model, sim_events[nonego], frame2distance_nonego, nonego)
    time_event_distance_illegal = model_to_events(
        model, sim_events['illegal'], frame2distance_illegal, 'illegal')

    return time_event_distance_ego, time_event_distance_nonego, time_event_distance_illegal


def smooth_trajectories(scenario, nonego,
                        trajectory_ego, trajectory_nonego,
                        frame2simDistance_ego, frame2simDistance_nonego, frame2simDistance_illegal,
                        time_event_distance_ego, time_event_distance_nonego, time_event_distance_illegal):
    """ Find:
    1. A realtime for each (ego, illegal, nonego) event distance s.t.
      (a) for each new agent, realtime is an increasing function of distance (no backing or teleportation)
      (b) the relative order of ruletimes of all events are preserved
    2. Cubic bezier interpolation between any two points (ts, ds) and (te, de)
      where ds, de are distances of two consecutive events of a car,
      and ts, te are the corresponding realtimes,
      and (ts+(te-ts)/3, d1) and (ts+2(te-ts)/3, d2) are intermediate control points, such that:
      (a) interpolation does not create new events
      (b) speed is continuous (to model no impact)
      (c) acceleration is bounded (to model bounded torque)
    """
    # Distances of events of a new logical time in increasing order
    from collections import OrderedDict
    logicalTime2distances_ego = OrderedDict()
    for time, _, distance in time_event_distance_ego:
        if not (time in logicalTime2distances_ego):
            logicalTime2distances_ego[time] = [distance]
        elif logicalTime2distances_ego[time][-1] < distance:
            logicalTime2distances_ego[time] += [distance]

    # Distances of events of a new ruletime in increasing order
    logicalTime2distances_nonego = OrderedDict()
    for time, _, distance in time_event_distance_nonego:
        if not (time in logicalTime2distances_nonego):
            logicalTime2distances_nonego[time] = [distance]
        elif logicalTime2distances_nonego[time][-1] < distance:
            logicalTime2distances_nonego[time] += [distance]

    # Distances of events of a new ruletime in increasing order
    logicalTime2distances_illegal = OrderedDict()
    for time, _, distance in time_event_distance_illegal:
        if not (time in logicalTime2distances_illegal):
            logicalTime2distances_illegal[time] = [distance]
        elif logicalTime2distances_illegal[time][-1] < distance:
            logicalTime2distances_illegal[time] += [distance]

    distances_ego = [round_down(d)
                     for ds in logicalTime2distances_ego.values()
                     for d in ds]
    distances_nonego = [round_down(d)
                        for ds in logicalTime2distances_nonego.values()
                        for d in ds]
    distances_illegal = [round_down(d)
                         for ds in logicalTime2distances_illegal.values()
                         for d in ds]

    import z3
    t_vars_ego = [0] + [z3.Real(f'T_ego_{i}')
                        for i in range(len(distances_ego))] + [scenario.maxSteps*scenario.timestep]
    t_vars_nonego = [0] + [z3.Real(f'T_nonego_{i}')
                           for i in range(len(distances_nonego))] + [scenario.maxSteps*scenario.timestep]
    t_vars_illegal = [0] + [z3.Real(f'T_illegal_{i}')
                            for i in range(len(distances_illegal))] + [scenario.maxSteps*scenario.timestep]

    d_vars_ego = [0 for i in range(len(t_vars_ego)*3-2)]
    d_vars_ego[-1] = round_down(frame2simDistance_ego[-1])
    for i in range(1, len(d_vars_ego)-1):
        if i % 3 == 0:     # interpolate event points (t, d)
            d_vars_ego[i] = distances_ego[i//3-1]
        else:
            d_vars_ego[i] = z3.Real(f'D_ego_{i//3}_{i%3}')

    d_vars_nonego = [0 for i in range(len(t_vars_nonego)*3-2)]
    d_vars_nonego[-1] = round_down(frame2simDistance_nonego[-1])
    for i in range(1, len(d_vars_nonego)-1):
        if i % 3 == 0:
            d_vars_nonego[i] = distances_nonego[i//3-1]
        else:
            d_vars_nonego[i] = z3.Real(f'D_nonego_{i//3}_{i%3}')

    d_vars_illegal = [0 for i in range(len(t_vars_illegal)*3-2)]
    d_vars_illegal[-1] = round_down(frame2simDistance_illegal[-1])
    for i in range(1, len(d_vars_illegal)-1):
        if i % 3 == 0:
            d_vars_illegal[i] = distances_illegal[i//3-1]
        else:
            d_vars_illegal[i] = z3.Real(f'D_illegal_{i//3}_{i%3}')

    constraints = []

    # 1. (a)
    constraints += [t_vars_ego[i] < t_vars_ego[i+1]
                    for i in range(len(t_vars_ego)-1)]
    constraints += [t_vars_nonego[i] < t_vars_nonego[i+1]
                    for i in range(len(t_vars_nonego)-1)]
    constraints += [t_vars_illegal[i] < t_vars_illegal[i+1]
                    for i in range(len(t_vars_illegal)-1)]

    # 1. (b)
    # Make a mapping from (agent, ruletime) to corresponding list of realtime variables.
    # Make a list (ruletime, agent) and sort it w.r.t ruletime.
    logicalTimes_ego = logicalTime2distances_ego.keys()
    logicalTime2ts_ego = {}
    for time in logicalTimes_ego:
        start = len(logicalTime2ts_ego.values())
        end = start + len(logicalTime2distances_ego[time])
        logicalTime2ts_ego[time] = t_vars_ego[1:-1][start:end]

    logicalTimes_nonego = logicalTime2distances_nonego.keys()
    logicalTime2ts_nonego = {}
    for time in logicalTimes_nonego:
        start = len(logicalTime2ts_nonego.values())
        end = start + len(logicalTime2distances_nonego[time])
        logicalTime2ts_nonego[time] = t_vars_nonego[1:-1][start:end]

    logicalTimes_illegal = logicalTime2distances_illegal.keys()
    logicalTime2ts_illegal = {}
    for time in logicalTimes_illegal:
        start = len(logicalTime2ts_illegal.values())
        end = start + len(logicalTime2distances_illegal[time])
        logicalTime2ts_illegal[time] = t_vars_illegal[1:-1][start:end]

    agent2ruletime2ts = {'ego': logicalTime2ts_ego,
                         'nonego': logicalTime2ts_nonego,
                         'illegal': logicalTime2ts_illegal}

    logicalTime_agent = [(r, 'ego') for r in logicalTimes_ego]
    logicalTime_agent += [(r, 'nonego') for r in logicalTimes_nonego]
    logicalTime_agent += [(r, 'illegal') for r in logicalTimes_illegal]

    # Add ruletimes of events of existing non-egos in scenario
    old_cars = {car for car in scenario.events.keys() if not car in {
        'ego', 'illegal'}}
    for car in old_cars:
        logicalTime_agent += [(frame_to_ruletime(e.frame, scenario.timestep), car)
                              for e in scenario.events[car]]

    logicalTime_agent.sort(key=lambda pair: pair[0])

    # 1. (b)
    # (i) If two consecutive ruletimes r1<r2 belong to different agents,
    #   then enforce ruletime(T1) < ruletime(T2)
    #   where T1,T2 are any realtimes associated with r1,r2 respectively.
    # (ii) If two consecutive ruletimes r1,r2 are equal (and so belong to different agents),
    #   then enforce ruletime(T1) = ruletime(T2)
    #   where T1,T2 are any realtimes associated with r1,r2 respectively.
    for i in range(len(logicalTime_agent)-1):
        r1, a1 = logicalTime_agent[i]
        r2, a2 = logicalTime_agent[i+1]
        # Ignore timing relations between events of same agent.
        # r1,r2 are constants for a1,a2 in old_cars so no constraints.
        if a1 == a2 or {a1, a2}.issubset(old_cars):
            continue
        # We have r1 <= r2 since ruletime_agent is sorted
        elif (r1 < r2):
            # ruletime(T) = int(2*T):
            T1 = r1 if a1 in old_cars else z3.ToInt(
                2*agent2ruletime2ts[a1][r1][-1])
            T2 = r2 if a2 in old_cars else z3.ToInt(
                2*agent2ruletime2ts[a2][r2][0])
            constraints += [T1 < T2]
        elif r1 == r2:
            T1 = r1 if a1 in old_cars else z3.ToInt(
                2*agent2ruletime2ts[a1][r1][0])
            T2 = r2 if a2 in old_cars else z3.ToInt(
                2*agent2ruletime2ts[a2][r2][-1])
            constraints += [T1 == T2]

    # TODO enforce max average velocity for stopping at an intersection

    # 2. (a)
    # ds <= d1 <= de, and ds <= d2 <= de
    for i in range(0, len(d_vars_ego)-3, 3):
        constraints += [d_vars_ego[i] <= d_vars_ego[i+1],
                        d_vars_ego[i+1] <= d_vars_ego[i+3],
                        d_vars_ego[i] <= d_vars_ego[i+2],
                        d_vars_ego[i+2] <= d_vars_ego[i+3]]
    for i in range(0, len(d_vars_nonego)-3, 3):
        constraints += [d_vars_nonego[i] <= d_vars_nonego[i+1],
                        d_vars_nonego[i+1] <= d_vars_nonego[i+3],
                        d_vars_nonego[i] <= d_vars_nonego[i+2],
                        d_vars_nonego[i+2] <= d_vars_nonego[i+3]]
    for i in range(0, len(d_vars_illegal)-3, 3):
        constraints += [d_vars_illegal[i] <= d_vars_illegal[i+1],
                        d_vars_illegal[i+1] <= d_vars_illegal[i+3],
                        d_vars_illegal[i] <= d_vars_illegal[i+2],
                        d_vars_illegal[i+2] <= d_vars_illegal[i+3]]

    # 2. (b)
    # Let dq < dr < ds be three consecutive distances,
    # tq < tr < ts be their corresponding realtimes,
    # dq1 and dq2 be the distances for tq+(tr-tq)/3 and tq+2(tr-tq)/3, and
    # dr1 and dr2 be the distances for tr+(ts-tr)/3 and tr+2(ts-tr)/3, respectively.
    # Then we require:
    # (tr-tq)(dr1-dr) = (ts-tr)(dr-dq2)
    for i in range(len(t_vars_ego)-2):
        tq, tr, ts = tuple(t_vars_ego[i:i+3])
        dq2, dr, dr1 = tuple(d_vars_ego[3*i+2:3*i+5])
        constraints += [(tr-tq)*(dr1-dr) == (ts-tr)*(dr-dq2)]

    for i in range(len(t_vars_nonego)-2):
        tq, tr, ts = tuple(t_vars_nonego[i:i+3])
        dq2, dr, dr1 = tuple(d_vars_nonego[3*i+2:3*i+5])
        constraints += [(tr-tq)*(dr1-dr) == (ts-tr)*(dr-dq2)]

    for i in range(len(t_vars_illegal)-2):
        tq, tr, ts = tuple(t_vars_illegal[i:i+3])
        dq2, dr, dr1 = tuple(d_vars_illegal[3*i+2:3*i+5])
        constraints += [(tr-tq)*(dr1-dr) == (ts-tr)*(dr-dq2)]

    # 2. (c)
    # Let am<0 and aM>0 be maximum deceleration and acceleration. Then we require
    # am <= 6(dr-2dr1+dr2)/(ts-tr)**2 <= aM and
    # am <= 6(dr1-2dr2+ds)/(ts-tr)**2 <= aM.
    # TODO move magic numbers to function arguments. Also, relation with average speed enforced in ASP.
    # am, aM = -10, 10
    # for i in range(len(t_vars_ego)-3):
    #     tr, ts = tuple(t_vars_ego[i:i+2])
    #     dr, dr1, dr2, ds = tuple(d_vars_ego[3*i:3*i+4])
    #     constraints += [am*(ts-tr)**2 <= 6*(dr-2*dr1+dr2),
    #                     6*(dr-2*dr1+dr2) <= aM*(ts-tr)**2,
    #                     am*(ts-tr)**2 <= 6*(dr1-2*dr2+ds),
    #                     6*(dr1-2*dr2+ds) <= aM*(ts-tr)**2]

    # for i in range(len(t_vars_nonego)-3):
    #     tr, ts = tuple(t_vars_nonego[i:i+2])
    #     dr, dr1, dr2, ds = tuple(d_vars_nonego[3*i:3*i+4])
    #     constraints += [am*(ts-tr)**2 <= 6*(dr-2*dr1+dr2),
    #                     6*(dr-2*dr1+dr2) <= aM*(ts-tr)**2,
    #                     am*(ts-tr)**2 <= 6*(dr1-2*dr2+ds),
    #                     6*(dr1-2*dr2+ds) <= aM*(ts-tr)**2]

    # for i in range(len(t_vars_illegal)-3):
    #     tr, ts = tuple(t_vars_illegal[i:i+2])
    #     dr, dr1, dr2, ds = tuple(d_vars_illegal[3*i:3*i+4])
    #     constraints += [am*(ts-tr)**2 <= 6*(dr-2*dr1+dr2),
    #                     6*(dr-2*dr1+dr2) <= aM*(ts-tr)**2,
    #                     am*(ts-tr)**2 <= 6*(dr1-2*dr2+ds),
    #                     6*(dr1-2*dr2+ds) <= aM*(ts-tr)**2]

    s = z3.Solver()
    s.add(constraints)
    print('Solving Z3 constraints...')
    print(s.check())

    # To convert Z3 rational numbers to Python's floating point reals
    def rat2fp(num):
        return float(num.numerator_as_long())/float(num.denominator_as_long())

    # Get the model
    m = s.model()

    t_ego = [0] + [rat2fp(m.eval(T))
                   for T in t_vars_ego[1:-1]] + [t_vars_ego[-1]]
    d_ego = [d_vars_ego[i] if i % 3 == 0 else rat2fp(m.eval(d_vars_ego[i]))
             for i in range(len(d_vars_ego))]

    t_nonego = [0] + [rat2fp(m.eval(T))
                      for T in t_vars_nonego[1:-1]] + [t_vars_nonego[-1]]
    d_nonego = [d_vars_nonego[i] if i % 3 == 0 else rat2fp(m.eval(d_vars_nonego[i]))
                for i in range(len(d_vars_nonego))]

    t_illegal = [0] + [rat2fp(m.eval(T))
                       for T in t_vars_illegal[1:-1]] + [t_vars_illegal[-1]]
    d_illegal = [d_vars_illegal[i] if i % 3 == 0 else rat2fp(m.eval(d_vars_illegal[i]))
                 for i in range(len(d_vars_illegal))]

    # TODO: update timings of new cars in scenario.events

    # Get interpolated points based on the Bezier control points
    from geomdl import BSpline

    # The new ego distance curve
    t_ego_comp = [t_ego[0]]
    for i in range(len(t_ego)-1):
        ts, te = t_ego[i], t_ego[i+1]
        ts1 = 2*ts/3 + te/3
        ts2 = ts/3 + 2*te/3
        t_ego_comp += [ts1, ts2, te]
    curve = BSpline.Curve()
    curve.degree = 3
    curve.ctrlpts = [[t_ego_comp[i], d_ego[i]] for i in range(len(d_ego))]
    kv = [0, 0, 0, 0]
    for i in range(1, len(t_ego)-1):
        kv += [t_ego[i], t_ego[i], t_ego[i]]
    kv += [t_ego[-1], t_ego[-1], t_ego[-1], t_ego[-1]]
    curve.knotvector = kv
    curve.sample_size = int(scenario.maxSteps)+1
    new2distance_ego = [p[1] for p in curve.evalpts]

    # The new nonego distance curve
    t_nonego_comp = [t_nonego[0]]
    for i in range(len(t_nonego)-1):
        ts, te = t_nonego[i], t_nonego[i+1]
        ts1 = 2*ts/3 + te/3
        ts2 = ts/3 + 2*te/3
        t_nonego_comp += [ts1, ts2, te]
    curve = BSpline.Curve()
    curve.degree = 3
    curve.ctrlpts = [[t_nonego_comp[i], d_nonego[i]]
                     for i in range(len(d_nonego))]
    kv = [0, 0, 0, 0]
    for i in range(1, len(t_nonego)-1):
        kv += [t_nonego[i], t_nonego[i], t_nonego[i]]
    kv += [t_nonego[-1], t_nonego[-1], t_nonego[-1], t_nonego[-1]]
    curve.knotvector = kv
    curve.sample_size = int(scenario.maxSteps)+1
    new2distance_nonego = [p[1] for p in curve.evalpts]

    # The new illegal distance curve
    t_illegal_comp = [t_illegal[0]]
    for i in range(len(t_illegal)-1):
        ts, te = t_illegal[i], t_illegal[i+1]
        ts1 = 2*ts/3 + te/3
        ts2 = ts/3 + 2*te/3
        t_illegal_comp += [ts1, ts2, te]
    curve = BSpline.Curve()
    curve.degree = 3
    curve.ctrlpts = [[t_illegal_comp[i], d_illegal[i]]
                     for i in range(len(d_illegal))]
    kv = [0, 0, 0, 0]
    for i in range(1, len(t_illegal)-1):
        kv += [t_illegal[i], t_illegal[i], t_illegal[i]]
    kv += [t_illegal[-1], t_illegal[-1], t_illegal[-1], t_illegal[-1]]
    curve.knotvector = kv
    curve.sample_size = int(scenario.maxSteps)+1
    new2distance_illegal = [p[1] for p in curve.evalpts]

    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3)
    fig.suptitle('distance-time curves for ego, nonego, illegal')
    axs[0].plot(frame2simDistance_ego, 'g')
    axs[0].plot(new2distance_ego)
    axs[0].scatter([realtime_to_frame_float(t, scenario.timestep) for t in t_ego_comp], d_ego,
                   c=['r' if i % 3 == 0 else 'b' for i in range(len(d_ego))],
                   s=[10 if i % 3 == 0 else 5 for i in range(len(d_ego))])
    axs[1].plot(frame2simDistance_nonego, 'g')
    axs[1].plot(new2distance_nonego)
    axs[1].scatter([realtime_to_frame_float(t, scenario.timestep) for t in t_nonego_comp], d_nonego,
                   c=['r' if i %
                       3 == 0 else 'b' for i in range(len(d_nonego))],
                   s=[10 if i % 3 == 0 else 5 for i in range(len(d_nonego))])
    axs[2].plot(frame2simDistance_illegal, 'g')
    axs[2].plot(new2distance_illegal)
    axs[2].scatter([realtime_to_frame_float(t, scenario.timestep) for t in t_illegal_comp], d_illegal,
                   c=['r' if i %
                       3 == 0 else 'b' for i in range(len(d_illegal))],
                   s=[10 if i % 3 == 0 else 5 for i in range(len(d_illegal))])
    plt.show()

    # The new ego trajectory
    new2old = []
    old = 0
    for new in range(len(trajectory_ego)):
        while old < len(trajectory_ego)-1 and frame2simDistance_ego[old] < new2distance_ego[new]:
            old += 1
        new2old += [old]

    new_traj_ego = []
    for frame in range(len(trajectory_ego)):
        new_traj_ego += [trajectory_ego[new2old[frame]]['ego']]

    # The new nonego trajectory
    new2old = []
    old = 0
    for new in range(len(trajectory_nonego)):
        while old < len(trajectory_nonego)-1 and frame2simDistance_nonego[old] < new2distance_nonego[new]:
            old += 1
        new2old += [old]

    new_traj_nonego = []
    for frame in range(len(trajectory_nonego)):
        new_traj_nonego += [trajectory_nonego[new2old[frame]][nonego]]

    # The new illegal trajectory
    new2old = []
    old = 0
    for new in range(len(trajectory_ego)):
        while old < len(trajectory_ego)-1 and frame2simDistance_illegal[old] < new2distance_illegal[new]:
            old += 1
        new2old += [old]

    new_traj_illegal = []
    for frame in range(len(trajectory_ego)):
        new_traj_illegal += [trajectory_ego[new2old[frame]]['ego']]

    # New timing of events
    t_e_d = time_event_distance_ego
    prev_d = t_e_d[0][2]
    j = 0
    for i in range(len(t_e_d)):
        if prev_d != t_e_d[i][2]:
            prev_d = t_e_d[i][2]
            j = j+1
        t_e_d[i][1].frame = realtime_to_frame(t_ego[j], scenario.timestep)

    t_e_d = time_event_distance_nonego
    prev_d = t_e_d[0][2]
    j = 0
    for i in range(len(t_e_d)):
        if prev_d != t_e_d[i][2]:
            prev_d = t_e_d[i][2]
            j = j+1
        t_e_d[i][1].frame = realtime_to_frame(t_nonego[j], scenario.timestep)

    t_e_d = time_event_distance_illegal
    prev_d = t_e_d[0][2]
    j = 0
    for i in range(len(t_e_d)):
        if prev_d != t_e_d[i][2]:
            prev_d = t_e_d[i][2]
            j = j+1
        t_e_d[i][1].frame = realtime_to_frame(t_illegal[j], scenario.timestep)

    new_trajs = {'ego': new_traj_ego,
                 nonego: new_traj_nonego,
                 'illegal': new_traj_illegal}

    new_events = {'ego': [ted[1] for ted in time_event_distance_ego],
                  nonego: [ted[1] for ted in time_event_distance_nonego],
                  'illegal': [ted[1] for ted in time_event_distance_illegal]}

    return new_trajs, new_events


def solution(scenario, sim_events,
             nonego, nonego_maneuver_uid, nonego_spawn_distance,
             sim_ego, sim_nonego,
             maxSpeed,
             extra_constraints):
    # All the given and new events
    import copy
    sim_events['illegal'] = []
    for event in sim_events['ego']:
        event_ill = copy.copy(event)
        event_ill.vehicle = 'illegal'
        sim_events['illegal'] += [event_ill]

    frame2simDistance_ego = frame_to_distance(sim_ego, 'ego')
    frame2simDistance_illegal = frame2simDistance_ego
    frame2simDistance_nonego = frame_to_distance(sim_nonego, nonego)

    l2e = logical_solution(scenario, sim_events,
                           nonego, nonego_maneuver_uid, nonego_spawn_distance,
                           sim_ego, sim_nonego,
                           frame2simDistance_ego, frame2simDistance_illegal, frame2simDistance_nonego,
                           maxSpeed,
                           extra_constraints)
    time_event_ego, time_event_nonego, time_event_illegal = l2e

    # Find trajectories that preserve the order of events in the logical solution
    new_trajs, new_events = smooth_trajectories(scenario, nonego,
                                                sim_ego.trajectory, sim_nonego.trajectory,
                                                frame2simDistance_ego, frame2simDistance_nonego, frame2simDistance_illegal,
                                                time_event_ego, time_event_nonego, time_event_illegal)

    traj_prev = scenario.trajectory
    # When extending an empty scenario
    if not traj_prev:
        traj_prev = [{} for i in range(len(new_trajs['ego']))]

    # Update the trajectory
    for frame in range(len(traj_prev)):
        traj_prev[frame]['ego'] = new_trajs['ego'][frame]
        traj_prev[frame][nonego] = new_trajs[nonego][frame]
        traj_prev[frame]['illegal'] = new_trajs['illegal'][frame]

    # Update the events
    sim_events.update(new_events)

    return traj_prev, sim_events


def extend(scenario, nonego_maneuver_uid,
           ego_spawn_distance=25,
           nonego_spawn_distance=10,
           nonego_blueprint='vehicle.tesla.model3',
           maxSpeed=8,
           extra_constraints=[]):
    import intersection_monitor
    monitor = intersection_monitor.Monitor()

    import scenic
    render = False

    params = {'map': scenario.map_path,
              'carla_map': scenario.map_name,
              'intersection_uid': scenario.intersection_uid,
              'timestep': scenario.timestep,
              'weather': scenario.weather,
              'render': render,
              'event_monitor': monitor}

    print('Sample an ego trajectory...')
    params['car_name'] = 'ego'
    params['maneuver_uid'] = scenario.maneuver_uid['ego']
    params['spawn_distance'] = ego_spawn_distance
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
    params['maneuver_uid'] = nonego_maneuver_uid
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
    new_traj, new_events = solution(
        scenario, monitor.events,
        nonego, nonego_maneuver_uid, nonego_spawn_distance,
        sim_result_ego, sim_result_nonego, maxSpeed,
        extra_constraints)

    from scenario import Scenario
    scenario_ext = Scenario()
    scenario_ext.maxSteps = scenario.maxSteps
    scenario_ext.timestep = scenario.timestep
    scenario_ext.weather = scenario.weather
    scenario_ext.map_path = scenario.map_path
    scenario_ext.map_name = scenario.map_name
    scenario_ext.intersection_uid = scenario.intersection_uid
    scenario_ext.rules_path = scenario.rules_path
    scenario_ext.blueprints = dict(
        scenario.blueprints, **{nonego: nonego_blueprint})
    scenario_ext.maneuver_uid = dict(
        scenario.maneuver_uid, **{nonego: nonego_maneuver_uid})
    scenario_ext.trajectory = new_traj
    scenario_ext.events = dict(scenario.events, **new_events)

    return scenario_ext
