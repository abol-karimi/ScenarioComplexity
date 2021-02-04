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


def model_to_events(model, events_all, car):
    """Returns a mapping from each new ruletime to corresponding events."""

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


def logical_solution(scenario, events_all,
                     nonego, nonego_maneuver_uid, nonego_spawn_distance,
                     sim_ego, sim_nonego,
                     frame2distance_ego,
                     frame2distance_illegal,
                     frame2distance_nonego,
                     maxSpeed,
                     extra_constraints):

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
                              events_all['ego'], frame2distance_ego, maxSpeed)
    atoms += traj_constraints(scenario,
                              events_all['illegal'], frame2distance_illegal, maxSpeed)
    atoms += traj_constraints(scenario,
                              events_all[nonego], frame2distance_nonego, maxSpeed)

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

    ruletime2events_ego = model_to_events(model, events_all, 'ego')
    ruletime2events_nonego = model_to_events(model, events_all, nonego)
    ruletime2events_illegal = model_to_events(model, events_all, 'illegal')

    return ruletime2events_ego, ruletime2events_nonego, ruletime2events_illegal


def events_to_trajectory(scenario,
                         ruletime2events,
                         car,
                         trajectory,
                         frame2distance,
                         ruletime2distances):
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
    plt.title(f'{car} trajectory')
    plt.plot(frame2distance, 'g')
    plt.plot(p_f, p_d, 'ro')
    plt.plot(new2distance)

    plt.xlabel('frame')
    plt.ylabel('distance')
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


def smooth_trajectories(self):
    # Realtime increases as distance increases along a trajectory.

    # Find:
    # 1. A realtime for each (ego, illegal, nonego) event distance s.t.
    #   the relative ruletime order of all events are preserved
    # 2. Two distances for each agent's time interval s.t.
    #   (a) distances are bounded by the distances at the interval ends.
    #   (b) distances correspond to (te-ts)/3 and 2(te-ts)/3 where [ts, te] is the interval
    #   (c) slope is continuous at interval endpoints

    # import z3
    # ruletime2realtimes = {}
    # for ruletime, distances in ruletime2distances.items():
    #     ruletime2realtimes[ruletime] = [
    #         z3.Real(f'T{ruletime}_{i}') for i in range(len(distances))]

    # # Z3 constraints
    # constraints = []

    # # Valid concretization
    # valid_concretization = []

    # ruletimes = sorted(ruletime2realtimes.keys())
    # # All the realtime variables in increasing order
    # realtimes_all = [realtime for ruletime in ruletimes
    #                  for realtime in ruletime2realtimes[ruletime]]
    # # Shorter distances have earlier realtimes
    # valid_concretization += [realtimes_all[i] < realtimes_all[i+1]
    #                          for i in range(len(realtimes_all)-1)]
    # # Concretizations of a ruletime preserves the ruletime
    # for ruletime, realtimes in ruletime2realtimes.items():
    #     for i in range(len(realtimes)-1):
    #         Ti = realtime_to_ruletime(realtimes[i])
    #         Tii = realtime_to_ruletime(realtimes[i+1])
    #         valid_concretization += [Ti == Tii]

    # # Order of ruletimes is preserved by the concretization
    # for i in range(len(ruletimes)-1):
    #     ti = ruletimes[i]
    #     tii = ruletimes[i+1]
    #     Ti = realtime_to_ruletime(ruletime2realtimes[ti][0])
    #     Tii = realtime_to_ruletime(ruletime2realtimes[tii][0])
    #     valid_concretization += [Ti < Tii]

    # constraints += valid_concretization

    # # Preserve logical solution (ego violating nonego's right-of-way)
    # logical_sol = []
    # max_ruletime = realtime_to_ruletime(self.max_realtime)
    # ego_ruletimes = sorted(set(
    #     [event.ruletime for event in self.events['ego']]))
    # import portion as P
    # ego_intervals = [P.closedopen(0, ego_ruletimes[0])]
    # for i in range(len(ego_ruletimes)-1):
    #     ti = ego_ruletimes[i]
    #     tii = ego_ruletimes[i+1]
    #     ego_intervals.append(P.singleton(ti))
    #     ego_intervals.append(P.open(ti, tii))
    # ego_intervals.append(P.singleton(ego_ruletimes[-1]))
    # ego_intervals.append(P.openclosed(ego_ruletimes[-1], max_ruletime))

    # for t in ruletimes:
    #     for I in ego_intervals:
    #         if t in I:
    #             T = realtime_to_ruletime(ruletime2realtimes[t][0])
    #             if I.lower == I.upper:
    #                 logical_sol += [T == t]
    #             elif I.left is P.OPEN and I.right is P.OPEN:
    #                 logical_sol += [I.lower < T, T < I.upper]
    #             elif I.left is P.CLOSED and I.right is P.OPEN:
    #                 logical_sol += [I.lower <= T, T < I.upper]
    #             elif I.left is P.OPEN and I.right is P.CLOSED:
    #                 logical_sol += [I.lower < T, T <= I.upper]
    #             break

    # constraints += logical_sol

    # # Interpolated points (ti, di)
    # event_distances = []
    # for ruletime in ruletimes:
    #     for distance in ruletime2distances[ruletime]:
    #         event_distances.append(distance)
    # print(event_distances)

    # interp_d = [0] + event_distances + [frame2distance[-1]]
    # interp_t = [0] + realtimes_all + [self.max_realtime]
    # print(interp_t)
    # print(interp_d)
    # # Smooth speed interpolation with bounded acceleration
    # # All the distances of events in increasing order
    # coeffs = [z3.Reals(f'a{i}_0 a{i}_1 a{i}_2')
    #           for i in range(len(interp_t)-1)]
    # interp = []
    # # The interpolation passes through each (ti, di).
    # # Non-negative speed at each ti so that
    # #  di <= d(t) <= dii for ti <= t <= tii.
    # for i in range(len(coeffs)):
    #     ti = interp_t[i]
    #     di = interp_d[i]
    #     tii = interp_t[i+1]
    #     dii = interp_d[i+1]
    #     a0, a1, a2 = coeffs[i]
    #     interp += [a2*ti**2 + a1*ti + a0 == di]
    #     interp += [a2*tii**2 + a1*tii + a0 == dii]
    #     interp += [2*a2*ti + a1 >= 0]
    # # Continuously differentiable
    # for i in range(len(coeffs)-1):
    #     tii = interp_t[i+1]
    #     _, a1, a2 = coeffs[i]
    #     _, b1, b2 = coeffs[i+1]
    #     interp += [2*a2*tii + a1 == 2*b2*tii + b1]
    # # Bound on acceleration (second derivative)
    # min_acceleration = -2
    # max_acceleration = 2
    # for _, _, a2 in coeffs:
    #     interp += [2*a2 <= max_acceleration]
    #     interp += [min_acceleration <= 2*a2]

    # constraints += interp

    # s = z3.Solver()
    # s.add(constraints)
    # print(s.check())

    # # To convert Z3 rational numbers to Python's floating point reals
    # def rat2fp(num):
    #     return float(num.numerator_as_long())/float(num.denominator_as_long())
    # m = s.model()
    # concrete_realtimes = [
    #     interp_t[0]] + [rat2fp(m.eval(T)) for T in interp_t[1:-1]] + [interp_t[-1]]
    # concrete_coeffs = [(rat2fp(m.eval(c[0])), rat2fp(m.eval(c[1])), rat2fp(m.eval(c[2])))
    #                    for c in coeffs]
    # # print(concrete_realtimes)
    # # print(concrete_coeffs)

    # nonego_intervals = []
    # for i in range(len(concrete_realtimes)-1):
    #     ti = concrete_realtimes[i]
    #     tii = concrete_realtimes[i+1]
    #     nonego_intervals += [P.closed(ti, tii)]

    # interval_index = 0
    # new2distance = []
    # for frame in range(len(trajectory)):
    #     t = frame * self.timestep
    #     while not t in nonego_intervals[interval_index]:
    #         interval_index += 1
    #     a0, a1, a2 = concrete_coeffs[interval_index]
    #     d = a2*(t**2) + a1*t + a0
    #     new2distance += [d]

    # import matplotlib.pyplot as plt
    # import numpy as np

    # fs = []
    # ds = []
    # for i in range(len(nonego_intervals)):
    #     I = nonego_intervals[i]
    #     a0, a1, a2 = concrete_coeffs[i]
    #     f = np.arange(I.lower, I.upper, self.timestep)
    #     fs += [t/self.timestep for t in f]
    #     ds += [a2*t**2 + a1*t + a0 for t in f]

    # plt.plot(fs, ds, '+')
    # plt.plot(frame2distance, '.')
    # plt.plot(new2distance, '.')

    # eventframe2distance = [[event.timestamp.frame, frame2distance[event.timestamp.frame]]
    #                        for event in self.events[self.nonego]]
    # plt.plot([p[0] for p in eventframe2distance], [p[1]
    #                                                for p in eventframe2distance], 'o')
    # plt.xlabel('frame')
    # plt.ylabel('distance')
    # plt.show()

    # new2old = []
    # old = 0
    # for new in range(len(trajectory)):
    #     while old < len(trajectory)-1 and frame2distance[old] < new2distance[new]:
    #         old += 1
    #     new2old += [old]

    # new_traj = []
    # for frame in range(len(trajectory)):
    #     new_traj += [trajectory[new2old[frame]][self.nonego]]
    # for frame in range(len(trajectory)):
    #     trajectory[frame][self.nonego] = new_traj[frame]

    return


def solution(scenario, events_all,
             nonego, nonego_maneuver_uid, nonego_spawn_distance,
             sim_ego, sim_nonego,
             maxSpeed,
             extra_constraints):
    import copy
    events_all['illegal'] = []
    for event in events_all['ego']:
        event_ill = copy.copy(event)
        event_ill.vehicle = 'illegal'
        events_all['illegal'] += [event_ill]

    frame2distance_ego = frame_to_distance(sim_ego, 'ego')
    frame2distance_illegal = frame2distance_ego
    frame2distance_nonego = frame_to_distance(sim_nonego, nonego)

    r2e = logical_solution(scenario, events_all,
                           nonego, nonego_maneuver_uid, nonego_spawn_distance,
                           sim_ego, sim_nonego,
                           frame2distance_ego, frame2distance_illegal, frame2distance_nonego,
                           maxSpeed,
                           extra_constraints)
    ruletime2events_ego, ruletime2events_nonego, ruletime2events_illegal = r2e

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

    # Distances of events of a ruletime in increasing order
    ruletime2distances_illegal = {}
    for ruletime, events in ruletime2events_illegal.items():
        distances = [frame2distance_illegal[event.frame]
                     for event in events]
        distances_sorted = sorted(set(distances))
        ruletime2distances_illegal[ruletime] = distances_sorted

    trajectory_ego = sim_ego.trajectory
    trajectory_nonego = sim_nonego.trajectory

    # Interpolate the events to a new trajectory
    new_traj_ego = events_to_trajectory(scenario,
                                        ruletime2events_ego,
                                        'ego',
                                        trajectory_ego,
                                        frame2distance_ego,
                                        ruletime2distances_ego)
    new_traj_nonego = events_to_trajectory(scenario,
                                           ruletime2events_nonego,
                                           nonego,
                                           trajectory_nonego,
                                           frame2distance_nonego,
                                           ruletime2distances_nonego)
    new_traj_illegal = events_to_trajectory(scenario,
                                            ruletime2events_illegal,
                                            'ego',
                                            trajectory_ego,
                                            frame2distance_illegal,
                                            ruletime2distances_illegal)

    traj_prev = scenario.trajectory
    # When extending an empty scenario
    if not traj_prev:
        traj_prev = [{} for i in range(len(trajectory_ego))]

    # Trajectories of the new vehicles
    for frame in range(len(traj_prev)):
        traj_prev[frame]['ego'] = new_traj_ego[frame]
        traj_prev[frame][nonego] = new_traj_nonego[frame]
        traj_prev[frame]['illegal'] = new_traj_illegal[frame]

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
    for ruletime, events in ruletime2events_illegal.items():
        ds = ruletime2distances_illegal[ruletime]
        for event in events:
            d = frame2distance_illegal[event.frame]
            fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
            frame = ruletime_to_frame(
                ruletime + fraction, scenario.timestep)
            event.frame = frame
    return traj_prev


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
    events = {car: event for car, event in scenario.events.items()}
    events.update(monitor.events)
    trajectory = solution(
        scenario, events,
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
    scenario_ext.blueprints = {nonego: nonego_blueprint}
    scenario_ext.blueprints.update(scenario.blueprints)
    scenario_ext.maneuver_uid = {nonego: nonego_maneuver_uid}
    scenario_ext.maneuver_uid.update(scenario.maneuver_uid)
    scenario_ext.trajectory = trajectory
    scenario_ext.events = events

    return scenario_ext
