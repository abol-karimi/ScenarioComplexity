from collections import OrderedDict
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


def realtime_to_frame(t, timestep):
    return int(realtime_to_frame_float(t, timestep))


def realtime_to_frame_float(t, timestep):
    return t/timestep


def frame_to_realtime(frame, timestep):
    return frame*timestep


def frame_to_distance(trajectory, car):
    frame2distance = [0]*len(trajectory)

    for i in range(len(trajectory)-1):
        pi = trajectory[i][car][0]
        pii = trajectory[i+1][car][0]
        frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

    return frame2distance


def distance_to_pose(distances, sim_distances, traj, car):
    """ For each frame, we are given a distance in 'sim_distances' and a corresponding pose in 'traj'.
    We return the poses corresponding to 'distances' by linear interpolation of the above (distance, pose) pairs.
    """
    ds, xs, ys, hs = [], [], [], []
    last_dist = -1
    for d, state in zip(sim_distances, traj):
        if last_dist == d:
            continue
        last_dist = d
        # add data points
        pose = state[car]
        x, y, h = pose[0].x, pose[0].y, pose[1]
        ds.append(d), xs.append(x), ys.append(y), hs.append(h)

    import numpy as np
    pi = np.pi
    xs_i = np.interp(distances, ds, xs)
    ys_i = np.interp(distances, ds, ys)
    hs_i = np.interp(distances, ds, np.unwrap(hs))
    # wrap headings back to (-pi,pi):
    hs_i = [(h + pi) % (2*pi) - pi for h in hs_i]

    from scenic.core.vectors import Vector
    poses = [[Vector(x, y), h] for x, y, h in zip(xs_i, ys_i, hs_i)]
    return poses


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


def realtime_logicalTime_axioms():
    atoms = []

    # If logically less-than, then realtime strictly less-than
    atoms += [f'realLTE(S, T) :- lessThan(S, T)',
              f':- lessThan(S, T), realLTE(T, S)']
    # If logically equal, then realtime equal
    atoms += [f'realLTE(S, T) :- equal(S, T)',
              f'realLTE(T, S) :- equal(S, T)']
    # Consistency of logical-time order
    atoms += [f':- lessThan(S, T), equal(S, T)']
    atoms += [f':- lessThan(S, T), equal(T, S)']

    # realLTE is a partial order
    atoms += [f'realLTE(T1, T3) :- realLTE(T1, T2), realLTE(T2, T3)',
              f':- realLTE(T1, T2), realLTE(T2, T1), T1 != T2, not equal(T1, T2), not equal(T2, T1)']

    return atoms


def car_to_time_to_events(sim_events):
    """Assumes that for each car, its events in sim_events are given in nondecreasing time order.
    Each car is mapped to a time2events dictionary.
    For each distinct frame in the frames of events of a car, a distince time constant is chosen.
    Each time constant is mapped to the list of corresponding events.
    """
    from collections import OrderedDict
    car2time2events = {car: OrderedDict() for car in sim_events.keys()}

    for car, events in sim_events.items():
        last_frame = -1
        i = -1
        for e in events:
            if e.frame != last_frame:
                i += 1
                t = f't_{car}_{i}'
                car2time2events[car][t] = [e]
                last_frame = e.frame
            else:
                car2time2events[car][t] += [e]

    return car2time2events


def logical_solution(scenario, sim_events, extra_constraints):
    """ Given the events for ego, nonego, and illegal (in 'sim_events')
    and their distances along the corresponding car's trajectory (in 'frame2distance_*'),
    find a timing for the events that satisfies the logical constraints.
    """

    atoms = extra_constraints

    # TODO store geometry atoms in the scenario to avoid computing them each time.
    from scenic.domains.driving.roads import Network
    network = Network.fromFile(scenario.map_path)
    atoms += geometry_atoms(network, scenario.intersection_uid)

    atoms += realtime_logicalTime_axioms()

    # Atoms of all events
    all_events = {car: events for car, events in scenario.events.items()}
    all_events.update(sim_events)
    car2time2events = car_to_time_to_events(all_events)
    for car, time2events in car2time2events.items():
        for t, events in time2events.items():
            atoms += [f'{e.withTime(t)}' for e in events]
    # For each car, a total order on its events
    for time2events in car2time2events.values():
        t = list(time2events.keys())
        for i in range(len(t)-1):
            atoms += [f'realLTE({t[i]}, {t[i+1]})',
                      f':- realLTE({t[i+1]}, {t[i]})']
    # The order of events of existing nonegos
    old_nonegos = {car for car in scenario.events if not car in {
        'ego', 'illegal'}}
    sym2val = {t: events[0].frame
               for car in old_nonegos for t, events in car2time2events[car].items()}
    atoms += [f'#script(python)\n'
              f'import clingo\n'
              f'sym2val = {sym2val}\n'
              f'min_perceptible_time = 10\n'  # frames
              f'def lessThan(S, T):\n'
              f'  lt = sym2val[S.name] + min_perceptible_time < sym2val[T.name]\n'
              f'  return clingo.Number(1) if lt else clingo.Number(0)\n'
              f'def equal(S, T):\n'
              f'  eq = abs(sym2val[S.name] - sym2val[T.name]) < min_perceptible_time\n'
              f'  return clingo.Number(1) if eq else clingo.Number(0)\n'
              f'#end']
    for s in sym2val:
        for t in sym2val:
            atoms += [f':- lessThan({s}, {t}), 0 = @lessThan({s}, {t})',
                      f':- equal({s}, {t}), 0 = @equal({s}, {t})']

    # Evidence that the new scenario has a solution
    atoms += [f':- V != illegal, violatesRightOf(ego, V)']
    atoms += [f':- violatesRule(ego, _)']

    # Evidence that the new scenario is strictly harder
    new_nonegos_pool = f'{"; ".join(car for car in sim_events if not car in {"ego", "illegal"})}'
    atoms += [
        f':- #count {{ 0:violatesRightOf(illegal, {new_nonegos_pool}) }} = 0']
    atoms += [f':- violatesRule(illegal, _)']
    for nonego in old_nonegos:
        atoms += [f':- violatesRightOf(illegal, {nonego})']

    # To generate unique time constants:
    atoms += [f'#script(python)\n'
              f'import clingo\n'
              f'def time(V, Pred, Tm, TM):\n'
              f'  t = V.name + Pred.name + Tm.name + TM.name\n'
              f'  return clingo.Function(t, [])\n'
              f'#end']

    # # The event StopppedAtForkAtTime is generated based on the status of traffic rules violations:
    atoms += [f'#count {{0:stoppedAtForkAtTime(V, F, @time(V, stop, T1, T2)); 1:lessThan(T1, @time(V, stop, T1, T2)); 2:lessThan(@time(V, stop, T1, T2),  T2) }} = 3 :-'
              f'arrivedAtForkAtTime(V, F, T1),'
              f'hasStopSign(F),'
              f'enteredForkAtTime(V, F, T2),'
              f'not violatesRule(V, stopAtSign)']

    from solver import Solver
    solver = Solver()
    solver.load(scenario.rules_path)
    solver.add_atoms(atoms)

    model = solver.solve()

    # Extract temporal constraints and the new events
    order_names = {'realLTE', 'lessThan', 'equal'}
    constraints = {n: set() for n in order_names}

    for atom in model:
        name = str(atom.name)
        if name in order_names:
            args = [str(a) for a in atom.arguments]
            constraints[name].add((args[0], args[1]))

    # Add events that are generated by the logic solver (not the physical simulation)
    from intersection_monitor import StoppedAtForkEvent
    for atom in model:
        if str(atom.name) != 'stoppedAtForkAtTime':
            continue
        car, fork, time = tuple([str(a) for a in atom.arguments])
        if not car in old_nonegos:
            car2time2events[car][time] = [StoppedAtForkEvent(car, fork, None)]

    # Sort the events by their realtime
    from functools import cmp_to_key

    def compare(s, t):
        LTE = constraints['realLTE']  # A total order for any fixed car
        lte, gte = (s, t) in LTE, (t, s) in LTE
        if lte and gte:
            return 0
        elif lte and not gte:
            return -1
        else:
            return 1
    for car, time2events in car2time2events.items():
        times = list(time2events.keys())
        times.sort(key=cmp_to_key(compare))
        car2time2events[car] = OrderedDict(
            [(t, time2events[t]) for t in times])

    return constraints, car2time2events


def smooth_trajectories(scenario, maxSpeed,
                        sim_trajectories,
                        temporal_constraints, car2time2events):
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
      (c) speed is small at stop events
      (d) bounds on speed
      (e) acceleration is bounded (to model bounded torque)
    """
    car2frame2simDistance = {car: frame_to_distance(
        sim_trajectories[car], car) for car in sim_trajectories}
    car2frame2simDistance['illegal'] = car2frame2simDistance['ego']

    new_vehicles = list(sim_trajectories.keys()) + ['illegal']

    time2distance = {}
    for car in new_vehicles:
        for t, events in car2time2events[car].items():
            f = events[0].frame
            d = car2frame2simDistance[car][f] if f != None else None
            time2distance[t] = round_down(d) if d != None else None

    car2distances = {car: [time2distance[t] for t in car2time2events[car]]
                     for car in new_vehicles}

    import z3
    maxTime = scenario.maxSteps*scenario.timestep
    t_list = {}
    for car in new_vehicles:
        var_list = [z3.Real(t) for t in car2time2events[car]]
        t_list[car] = [0] + var_list + [maxTime]

    # Index z3 time variables by their name
    t2var = {}
    for ts in t_list.values():
        t2var.update({str(t): t for t in ts[1:-1]})
    old_nonegos = {car for car in scenario.events if not car in {
        'ego', 'illegal'}}
    t2var.update({t: frame_to_realtime(events[0].frame, scenario.timestep)
                  for car in old_nonegos for t, events in car2time2events[car].items()})
    d_list = {}
    for car in new_vehicles:
        d_list[car] = [0 for i in range(len(t_list[car])*3-2)]
        d_list[car][-1] = round_down(car2frame2simDistance[car][-1])
        for i in range(1, len(d_list[car])-1):
            if i % 3 == 0:  # Interpolation points
                d_list[car][i] = car2distances[car][i//3-1]
                if d_list[car][i] == None:  # no distance constraint for this event
                    d_list[car][i] = z3.Real(f'D_{car}_{i//3}')
            else:
                d_list[car][i] = z3.Real(f'D_{car}_{i//3}_{i%3}')

    constraints = []

    # 1.
    min_perceptible_time = 0.5  # seconds

    for car in new_vehicles:
        constraints += [s < t for s, t in zip(t_list[car], t_list[car][1:])]
    constraints += [min_perceptible_time <= t2var[t] - t2var[s]
                    for s, t in temporal_constraints['lessThan']]
    constraints += [-min_perceptible_time < t2var[t] - t2var[s]
                    for s, t in temporal_constraints['equal']]
    constraints += [t2var[t] - t2var[s] < min_perceptible_time
                    for s, t in temporal_constraints['equal']]
    # 2. (a)
    # ds <= d1 <= de, and ds <= d2 <= de
    for car in new_vehicles:
        for i in range(0, len(d_list[car])-3, 3):
            constraints += [d_list[car][i] <= d_list[car][i+1],
                            d_list[car][i+1] <= d_list[car][i+3],
                            d_list[car][i] <= d_list[car][i+2],
                            d_list[car][i+2] <= d_list[car][i+3]]

    # 2. (b)
    # Let dq < dr < ds be three consecutive distances,
    # tq < tr < ts be their corresponding realtimes,
    # dq1 and dq2 be the distances for tq+(tr-tq)/3 and tq+2(tr-tq)/3, and
    # dr1 and dr2 be the distances for tr+(ts-tr)/3 and tr+2(ts-tr)/3, respectively.
    # Then we require:
    # (tr-tq)(dr1-dr) = (ts-tr)(dr-dq2)
    for car in new_vehicles:
        for i in range(len(t_list[car])-2):
            tq, tr, ts = tuple(t_list[car][i:i+3])
            dq2, dr, dr1 = tuple(d_list[car][3*i+2:3*i+5])
            constraints += [(tr-tq)*(dr1-dr) == (ts-tr)*(dr-dq2)]

    # 2. (c)
    # TODO if no stoppedAtFork event, force a minimum speed
    stop_speed = 1.0  # meters/second

    for car in new_vehicles:
        for i in range(1, len(t_list[car])-1):
            e_names = [e.name for e in car2time2events[car]
                       [str(t_list[car][i])]]
            if 'stoppedAtForkAtTime' in e_names:
                delta_t = (t_list[car][i+1] - t_list[car][i])/3
                delta_d = d_list[car][3*i+1] - d_list[car][3*i]
                constraints += [delta_d/delta_t < stop_speed]

    # 2. (d)
    for car in new_vehicles:
        for i in range(len(t_list[car])-1):
            tq, tr = tuple(t_list[car][i:i+2])
            dq, dq1, dq2, dr = tuple(d_list[car][3*i:3*i+4])
            # constraints += [3*(dq1-dq)/(tr-tq) <= maxSpeed,
            #                 3*(dr-dq2)/(tr-tq) <= maxSpeed]  # instantaneous speed
            constraints += [(dr-dq)/(tr-tq) <= maxSpeed]  # average speed

    # 2. (e)
    # Let am<0 and aM>0 be maximum deceleration and acceleration. Then we require
    # am <= 6(dr-2dr1+dr2)/(ts-tr)**2 <= aM and
    # am <= 6(dr1-2dr2+ds)/(ts-tr)**2 <= aM.
    # TODO move magic numbers to function arguments.
    # am, aM = -10, 10
    # for car in new_vehicles:
    #     for i in range(len(t_list[car])-3):
    #         tr, ts = tuple(t_list[car][i:i+2])
    #         dr, dr1, dr2, ds = tuple(d_list[car][3*i:3*i+4])
    #         constraints += [am*(ts-tr)**2 <= 6*(dr-2*dr1+dr2),
    #                         6*(dr-2*dr1+dr2) <= aM*(ts-tr)**2,
    #                         am*(ts-tr)**2 <= 6*(dr1-2*dr2+ds),
    #                         6*(dr1-2*dr2+ds) <= aM*(ts-tr)**2]

    s = z3.Solver()
    s.set(unsat_core=True)
    for c in constraints:
        s.assert_and_track(c, str(c))
    print('Solving smoothness constraints...')
    print(s.check())
    unsat_core = s.unsat_core()
    if len(unsat_core) > 0:
        print('unsat_core: ', unsat_core)

    # To convert Z3 rational numbers to Python's floating point reals
    def rat2fp(num):
        return float(num.numerator_as_long())/float(num.denominator_as_long())

    # Get the model
    m = s.model()

    t, d = {}, {}
    for car in new_vehicles:
        t[car] = [0] + [rat2fp(m.eval(T))
                        for T in t_list[car][1:-1]] + [t_list[car][-1]]
        d[car] = [rat2fp(m.eval(d)) if isinstance(d, z3.ExprRef) else d
                  for d in d_list[car]]

    # Get interpolated points based on the Bezier control points
    from geomdl import BSpline

    t_comp = {}
    new2distance = {}
    for car in new_vehicles:
        # The new ego distance curve
        t_comp[car] = [t[car][0]]
        for i in range(len(t[car])-1):
            ts, te = t[car][i], t[car][i+1]
            ts1 = 2*ts/3 + te/3
            ts2 = ts/3 + 2*te/3
            t_comp[car] += [ts1, ts2, te]
        curve = BSpline.Curve()
        curve.degree = 3
        curve.ctrlpts = [[t_comp[car][i], d[car][i]]
                         for i in range(len(d[car]))]
        kv = [0, 0, 0, 0]
        for i in range(1, len(t[car])-1):
            kv += [t[car][i], t[car][i], t[car][i]]
        kv += [t[car][-1], t[car][-1], t[car][-1], t[car][-1]]
        curve.knotvector = kv
        curve.sample_size = int(scenario.maxSteps)+1
        new2distance[car] = [p[1] for p in curve.evalpts]

    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(new_vehicles))
    fig.suptitle('distance-time curves for ego, illegal, and nonegos')
    for j, car in enumerate(new_vehicles):
        axs[j].plot(new2distance[car])
        axs[j].scatter([realtime_to_frame_float(t, scenario.timestep) for t in t_comp[car]], d[car],
                       c=['r' if i %
                           3 == 0 else 'b' for i in range(len(d[car]))],
                       s=[10 if i % 3 == 0 else 5 for i in range(len(d[car]))])
    plt.show()

    # The new trajectories
    new_traj = {}
    for car in new_vehicles:
        alias = 'ego' if car == 'illegal' else car
        new_traj[car] = distance_to_pose(
            new2distance[car], car2frame2simDistance[car], sim_trajectories[alias], alias)

    # New timing of events
    for car in new_vehicles:
        for tvar, tval in zip(t_list[car][1:-1], t[car][1:-1]):
            frame = realtime_to_frame(tval, scenario.timestep)
            for e in car2time2events[car][str(tvar)]:
                e.frame = frame

    new_events = {}
    for car in new_vehicles:
        new_events[car] = []
        for es in car2time2events[car].values():
            new_events[car] += es

    return new_traj, new_events


def solution(scenario, sim_events,
             nonego, nonego_maneuver_uid, nonego_spawn_distance,
             sim_trajectories,
             maxSpeed,
             extra_constraints):
    # All the given and new events
    import copy
    sim_events['illegal'] = []
    for event in sim_events['ego']:
        event_ill = copy.copy(event)
        event_ill.vehicle = 'illegal'
        sim_events['illegal'] += [event_ill]

    constraints, car2time2events = logical_solution(
        scenario, sim_events, extra_constraints)

    # Find trajectories that preserve the order of events in the logical solution
    new_trajs, new_events = smooth_trajectories(scenario, maxSpeed,
                                                sim_trajectories,
                                                constraints, car2time2events)

    print('Solution events:')
    for events in new_events.values():
        for e in events:
            print(f'\t{e.withTime(e.frame)}')
        print('')

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

    print('Simulate an ego trajectory...')
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

    print('Simulate a nonego trajectory...')
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
    sim_trajectories = {'ego': sim_result_ego.trajectory,
                        nonego: sim_result_nonego.trajectory}
    new_traj, new_events = solution(
        scenario, monitor.events,
        nonego, nonego_maneuver_uid, nonego_spawn_distance,
        sim_trajectories, maxSpeed,
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
