from collections import OrderedDict
import math
from solver import NoASPSolutionError
from utils import has_collision, frame_to_distance

from pysmt.logics import QF_NRA
from pysmt.shortcuts import get_env, Solver, get_model, Symbol, Equals, And, Real
from pysmt.typing import REAL
import pysmt
import fractions
import clingo

solver_name = "z3-binary"
path = ["z3", "-in", "-smt2"]  # tested with z3-4.8.10-x64-ubuntu-18.04

# solver_name = "mathsat-binary"
# path = ["mathsat"] # tested with mathsat-5.6.6-linux-x86_64

# solver_name = "cvc4-binary"
# path = ["~/Downloads/cvc4-1.8-x86_64-linux-opt",
#         "--lang=smt2", "--produce-models", "--no-interactive-prompt"]

# solver_name = "yices-binary"
# path = ["yices-smt2"]

logics = [QF_NRA]
env = get_env()
env.factory.add_generic_solver(solver_name, path, logics)


class NoSMTSolutionError(Exception):
    """Exception raised for errors in SMT solving.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, message):
        self.message = message


def numeral_to_fp(num):
    if isinstance(num, fractions.Fraction):
        return num.numerator/num.denominator
    elif isinstance(num, pysmt.constants.Numeral):
        rat = num.approx()
        fr = rat.as_fraction()
        return fr.numerator/fr.denominator
    else:
        print('Incompatible type: ', num, type(num))
        return None


# Rounds r >=0 down to precision number of decimal places.
def round_up(r, precision=3):
    coeff = 10**precision
    return fractions.Fraction(math.ceil(r*coeff), coeff)


# Rounds r >=0 down to precision number of decimal places.
def round_down(r, precision=3):
    coeff = 10**precision
    return fractions.Fraction(math.floor(r*coeff), coeff)


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


def distance_to_time(t, d, d_val):
    """t, d: array of coordinates of the control points of the composite Bezier curve.
    d_val: the d-value to at which the curve is projected to the t-axis.
    returns the projection.
    """
    i = 0
    while d[3*i+3] < d_val:
        i += 1
    # When here, d[3*i] <= d_val <= d[3*i+3]
    if d_val == d[3*i]:
        return t[i]
    elif d_val == d[3*i+3]:
        return t[i+1]

    # TODO use a numerical solver
    t_var = Symbol('t', REAL)
    b0 = (1-t_var)*(1-t_var)*(1-t_var)
    b1 = 3*t_var*(1-t_var)*(1-t_var)
    b2 = 3*t_var*t_var*(1-t_var)
    b3 = t_var*t_var*t_var

    constraints = [t_var > 0,
                   t_var < 1,
                   Equals(Real(d_val), d[3*i]*b0 + d[3*i+1]*b1 + d[3*i+2]*b2 + d[3*i+3]*b3)]

    t_global = None
    with Solver(name=solver_name, logic=QF_NRA):
        m = get_model(And(constraints))
        numeral = m.get_py_value(t_var)
        t_local = numeral_to_fp(numeral)
        t_global = (1-t_local)*t[i] + t_local*t[i+1]

    return t_global


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
        ii = (i+1) % len(roads)  # cyclic order
        lefts = road2incomings[roads[i].uid]
        rights = road2incomings[roads[ii].uid]
        l0 = network.elements[lefts[0]]
        r0 = network.elements[rights[0]]
        hl = l0.centerline[-1] - l0.centerline[-2]  # heading
        hr = r0.centerline[-1] - r0.centerline[-2]  # heading
        # Ignore roads on opposing directions:
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

    # If perceptibly less-than, then realtime strictly less-than
    atoms += [f'realLTE(S, T) :- lessThan(S, T)',
              f':- lessThan(S, T), realLTE(T, S)']
    # If perceptibly equal, then realtime equal
    atoms += [f'realLTE(S, T) :- equal(S, T)',
              f'realLTE(T, S) :- equal(S, T)']
    # Internal consistency of perceptible order
    atoms += [f':- lessThan(S, T), equal(S, T)']
    atoms += [f':- lessThan(S, T), equal(T, S)']

    # realLTE is a partial order
    atoms += [f'realLTE(T1, T3) :- realLTE(T1, T2), realLTE(T2, T3)',
              f':- realLTE(T1, T2), realLTE(T2, T1), T1 != T2, not equal(T1, T2), not equal(T2, T1)']

    return atoms


def car_to_time_to_events(car2events):
    """Assumes that for each car, its events in car2events are given in nondecreasing time order.
    Each car is mapped to a time2events dictionary.
    For each distinct frame in the frames of events of a car, a distince time constant is chosen.
    Each time constant is mapped to the list of corresponding events.
    """
    from collections import OrderedDict
    car2time2events = {car: OrderedDict() for car in car2events.keys()}

    for car, events in car2events.items():
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


def logical_solution(scenario, config, sim_events):
    """ Given the events for ego, nonego, and illegal (in 'sim_events')
    and their distances along the corresponding car's trajectory (in 'frame2distance_*'),
    find a timing for the events that satisfies the logical constraints.
    """

    atoms = []
    atoms += config['constraints']

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
    min_perceptible_time = 10  # frames
    atoms += [f'#script(python)\n'
              f'import clingo\n'
              f'sym2val = {sym2val}\n'
              f'def lessThan(S, T):\n'
              f'  lt = sym2val[S.name] + {min_perceptible_time} < sym2val[T.name]\n'
              f'  return clingo.Number(1) if lt else clingo.Number(0)\n'
              f'def equal(S, T):\n'
              f'  eq = abs(sym2val[S.name] - sym2val[T.name]) < {min_perceptible_time}\n'
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
    new_nonegos = [car for car in sim_events if not car in {'ego', 'illegal'}]
    atoms += [':- ' +
              ', '.join(f'not violatesRightOf(illegal, {car})' for car in new_nonegos)]
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

    # The event StopppedAtForkAtTime is generated for new cars,
    #  based on the status of traffic rules violations:
    for car in new_nonegos+['ego', 'illegal']:
        atoms += [f'#count {{0:stoppedAtForkAtTime({car}, F, @time({car}, stop, T1, T2)); 1:lessThan(T1, @time({car}, stop, T1, T2)); 2:lessThan(@time({car}, stop, T1, T2),  T2) }} = 3 :-'
                  f'arrivedAtForkAtTime({car}, F, T1),'
                  f'hasStopSign(F),'
                  f'enteredForkAtTime({car}, F, T2),'
                  f'not violatesRule({car}, stopAtSign)']

    program = ""
    for atom in atoms:
        program += f'{atom}.\n'

    ctl = clingo.Control()
    ctl.load(scenario.rules_path)
    ctl.add("base", [], program)
    ctl.ground([("base", [])])
    ctl.configuration.solve.models = "10000"
    models = []
    with ctl.solve(yield_=True) as handle:
        for model in handle:
            models.append(model.symbols(atoms=True))

    print(f'Number of ASP models found: {len(models)}')

    return models, car2time2events


def model_to_constraints(model, car2time2events, old_nonegos):

    # Extract temporal constraints and the new events
    order_names = {'realLTE', 'lessThan', 'equal'}
    constraints = {n: set() for n in order_names}
    constraints['stop'] = set()

    from intersection_monitor import StoppedAtForkEvent
    for atom in model:
        name = str(atom.name)
        if name in order_names:
            args = [str(a) for a in atom.arguments]
            constraints[name].add((args[0], args[1]))
        # Events that are generated by the solver
        if name == 'stoppedAtForkAtTime':
            car, fork, time = tuple([str(a) for a in atom.arguments])
            if not car in old_nonegos:
                constraints['stop'].add(time)
                if time in car2time2events[car]:
                    car2time2events[car][time].append(
                        StoppedAtForkEvent(car, fork, None))
                else:
                    car2time2events[car][time] = [
                        StoppedAtForkEvent(car, fork, None)]

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


def smooth_trajectories(scenario, config,
                        sim_trajectories,
                        temporal_constraints,
                        car2time2events):
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
    car2frame2simDistance = {car: frame_to_distance(sim_trajectories[car])
                             for car in sim_trajectories}

    new_cars = list(sim_trajectories.keys())

    t_dom = set()
    for s, t in temporal_constraints['lessThan']:
        t_dom.update({s, t})
    for s, t in temporal_constraints['equal']:
        t_dom.update({s, t})
    t_dom.update(temporal_constraints['stop'])

    time2distance = {}
    for car in new_cars:
        for t, events in car2time2events[car].items():
            if t in t_dom:
                f = events[0].frame
                time2distance[t] = car2frame2simDistance[car][f] if f != None else None

    car2distances = {car: [time2distance[t] for t in car2time2events[car] if t in t_dom]
                     for car in new_cars}

    maxTime = scenario.maxSteps*scenario.timestep
    t_list = {}
    for car in new_cars:
        var_list = [Symbol(t, REAL)
                    for t in car2time2events[car] if t in t_dom]
        t_list[car] = [Real(0)] + var_list + [Real(maxTime)]

    d_list = {}
    for car in new_cars:
        d_list[car] = [0 for i in range(len(t_list[car])*3-2)]
        d_list[car][0] = Real(0)
        d_list[car][-1] = Real(round_down(car2frame2simDistance[car][-1]))
        for i in range(1, len(d_list[car])-1):
            if i % 3 == 0:  # Interpolation point
                di = car2distances[car][i//3-1]
                if di == None:  # no distance constraint for this event
                    d_list[car][i] = Symbol(f'D_{car}_{i//3}', REAL)
                else:
                    d_list[car][i] = Real(round_down(di))
            else:  # Bezier control point
                d_list[car][i] = Symbol(f'D_{car}_{i//3}_{i%3}', REAL)

    # Index realtime variables by their name
    old_nonegos = {car for car in scenario.events if not car in {
        'ego', 'illegal'}}
    t2var = {}
    for ts in t_list.values():
        t2var.update({str(t): t for t in ts[1:-1]})
    t2var.update({t: Real(round_down(frame_to_realtime(events[0].frame, scenario.timestep)))
                  for car in old_nonegos for t, events in car2time2events[car].items() if t in t_dom})

    # Add extra control points to increase the flexibility of the curve
    max_separation = 30.
    t_list_augmented = {}
    d_list_augmented = {}
    for car in new_cars:
        t_list_augmented[car] = []
        d_list_augmented[car] = []
        for i in range(len(t_list[car])-1):
            ti = t_list[car][i]
            di, di_0, di_1, dii = d_list[car][3*i:3*i+4]

            t_list_augmented[car] += [ti]
            d_list_augmented[car] += [di, di_0, di_1]

            if not (di.is_constant() and dii.is_constant()):
                continue

            di_fp = numeral_to_fp(di.constant_value())
            dii_fp = numeral_to_fp(dii.constant_value())
            if dii_fp-di_fp <= max_separation:
                continue
            n = math.ceil((dii_fp-di_fp)/max_separation)
            separation = (dii_fp-di_fp)/n
            t_base = f'{car}_{ti}' if ti.is_constant() else ti
            t_list_augmented[car] += [Symbol(f'{t_base}_aug_{j}', REAL)
                                      for j in range(n-1)]
            for j in range(3*(n-1)):
                if j % 3 == 0:
                    d_list_augmented[car] += [
                        Real(round_down(di_fp+(j//3+1)*separation))]
                else:
                    d_list_augmented[car] += [
                        Symbol(f'D_{car}_{i}_aug_{j}', REAL)]
        t_list_augmented[car] += [t_list[car][-1]]
        d_list_augmented[car] += [d_list[car][-1]]
    t_list = t_list_augmented
    d_list = d_list_augmented

    constraints = []

    # 1.
    min_perceptible_time = 0.5  # seconds

    for car in new_cars:
        constraints += [s < t for s, t in zip(t_list[car], t_list[car][1:])]
    constraints += [min_perceptible_time*2 <= t2var[t] - t2var[s]  # *2 to get a crisper constrast
                    for s, t in temporal_constraints['lessThan']]
    constraints += [-min_perceptible_time < t2var[t] - t2var[s]
                    for s, t in temporal_constraints['equal']]
    constraints += [t2var[t] - t2var[s] < min_perceptible_time
                    for s, t in temporal_constraints['equal']]
    # 2. (a)
    # ds <= d1 <= de, and ds <= d2 <= de
    for car in new_cars:
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
    for car in new_cars:
        for i in range(len(t_list[car])-2):
            tq, tr, ts = tuple(t_list[car][i:i+3])
            dq2, dr, dr1 = tuple(d_list[car][3*i+2:3*i+5])
            constraints += [Equals((tr-tq)*(dr1-dr), (ts-tr)*(dr-dq2))]

    # 2. (c)
    # TODO if no stoppedAtFork event, force a minimum speed
    stop_speed = 1.0  # meters/second

    for car in new_cars:
        for i in range(1, len(t_list[car])-1):
            if str(t_list[car][i]) in temporal_constraints['stop']:
                delta_t = (t_list[car][i+1] - t_list[car][i])/3
                delta_d = d_list[car][3*i+1] - d_list[car][3*i]
                constraints += [delta_d/delta_t < Real(stop_speed)]

    # 2. (d)
    for car in new_cars:
        maxSpeed = config[car]['maxSpeed']
        for i in range(len(t_list[car])-1):
            tq, tr = tuple(t_list[car][i:i+2])
            dq, dq1, dq2, dr = tuple(d_list[car][3*i:3*i+4])
            # constraints += [3*(dq1-dq)/(tr-tq) <= maxSpeed,
            #                 3*(dr-dq2)/(tr-tq) <= maxSpeed]  # instantaneous speed
            constraints += [(dr-dq)/(tr-tq) <= Real(maxSpeed)]  # average speed

    # 2. (e)
    # Let am<0 and aM>0 be maximum deceleration and acceleration. Then we require
    # am <= 6(dr-2dr1+dr2)/(ts-tr)**2 <= aM and
    # am <= 6(dr1-2dr2+ds)/(ts-tr)**2 <= aM.
    # for car in new_cars:
    #     am = Real(config[car]['minAcceleration']) # -8 is used in RSS
    #     aM = Real(config[car]['maxAcceleration']) # 4 is used in RSS
    #     for i in range(len(t_list[car])-3):
    #         tr, ts = tuple(t_list[car][i:i+2])
    #         dr, dr1, dr2, ds = tuple(d_list[car][3*i:3*i+4])
    #         constraints += [am*(ts-tr)*(ts-tr) <= 6*(dr-2*dr1+dr2),
    #                         6*(dr-2*dr1+dr2) <= aM*(ts-tr)*(ts-tr),
    #                         am*(ts-tr)*(ts-tr) <= 6*(dr1-2*dr2+ds),
    #                         6*(dr1-2*dr2+ds) <= aM*(ts-tr)*(ts-tr)]

    # Collision-avoidance
    # min_dist = 2
    # for i in range(len(new_cars)-1):
    #     for j in range(i+1, len(new_cars)):
    #         car1, car2 = new_cars[i], new_cars[j]
    #         if car1 == 'illegal' or car2 == 'illegal':
    #             continue
    #         for m in range(1, len(t_list[car1])-1):
    #             for n in range(1, len(t_list[car2])-1):
    #                 dm, dn = d_list[car1][3*m], d_list[car2][3*n]
    #                 if type(dm) != int and type(dm) != float:
    #                     continue
    #                 if type(dn) != int and type(dn) != float:
    #                     continue
    #                 tm, tn = t_list[car1][m], t_list[car2][n]
    #                 f1 = car2time2events[car1][str(tm)][0].frame
    #                 f2 = car2time2events[car2][str(tn)][0].frame
    #                 pose1 = sim_trajectories[car1][f1][car1]
    #                 pose2 = sim_trajectories[car2][f2][car2]
    #                 x1, y1 = pose1[0].x, pose1[0].y
    #                 x2, y2 = pose2[0].x, pose2[0].y
    #                 dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
    #                 if dist < min_dist:
    #                     delta = round_down(1/(min_dist - dist))
    #                     print(
    #                         f'Collision constraint: {z3.Or(tm-tn > delta, tm-tn < -delta)}')
    #                     constraints += [z3.Or(tm-tn > delta, tm-tn < -delta)]

    with Solver(name=solver_name, logic=QF_NRA):
        m = get_model(And(constraints))
        if m == None:
            raise NoSMTSolutionError(
                f'SMT solver {solver_name} found no solutions.')

    t, d = {}, {}
    for car in new_cars:
        t[car] = [numeral_to_fp(m.get_py_value(T)) for T in t_list[car]]
        d[car] = [numeral_to_fp(m.get_py_value(D)) for D in d_list[car]]

    # Store parameters of the composite Bezier curves
    curves = {}
    for car in new_cars:
        curves[car] = {}
        # car's new time-distance curve
        t_comp = [t[car][0]]
        for i in range(len(t[car])-1):
            ts, te = t[car][i], t[car][i+1]
            ts1 = 2*ts/3 + te/3
            ts2 = ts/3 + 2*te/3
            t_comp += [ts1, ts2, te]
        curves[car]['degree'] = 3
        curves[car]['ctrlpts'] = [[t_comp[i], d[car][i]]
                                  for i in range(len(d[car]))]
        kv = [0, 0, 0, 0]
        for i in range(1, len(t[car])-1):
            kv += [t[car][i], t[car][i], t[car][i]]
        kv += [t[car][-1], t[car][-1], t[car][-1], t[car][-1]]
        curves[car]['knotvector'] = kv

    # New timing of events
    for car in new_cars:
        for time, events in car2time2events[car].items():
            f = events[0].frame
            if f != None:
                d_sim = car2frame2simDistance[car][f]
                t_new = distance_to_time(t[car], d[car], d_sim)
            else:
                t_new = numeral_to_fp(m.get_py_value(t2var[time]))
            for e in events:
                e.frame = realtime_to_frame(t_new, scenario.timestep)

    new_events = {}
    for car in new_cars:
        new_events[car] = []
        for es in car2time2events[car].values():
            new_events[car] += es

    return new_events, curves


def solution(scenario, config,
             sim_events,
             sim_trajectories,
             car_sizes):
    # All the given and new events
    import copy
    sim_events['illegal'] = []
    for event in sim_events['ego']:
        event_ill = copy.copy(event)
        event_ill.vehicle = 'illegal'
        sim_events['illegal'] += [event_ill]

    models, car2time2events = logical_solution(scenario, config, sim_events)

    if len(models) == 0:
        raise NoASPSolutionError('No ASP solution found!')

    # from scenic.domains.driving.roads import Network
    # from visualization import draw_intersection
    # import carla
    # client = carla.Client('127.0.0.1', 2000)
    # world = client.load_world(scenario.map_name)
    # network = Network.fromFile(scenario.map_path)
    # intersection = network.elements[scenario.intersection_uid]
    # draw_intersection(world, intersection)

    # Find trajectories that preserve the order of events in the logical solution
    import copy
    old_nonegos = {car for car in scenario.events if not car in {
        'ego', 'illegal'}}
    new_events = None
    for i, model in enumerate(models):
        constraints, car2time2events_updated = model_to_constraints(
            model, copy.deepcopy(car2time2events), old_nonegos)

        print(f'Generating smooth trajecotries for {i}th ASP model...')
        try:
            new_events, curves = smooth_trajectories(scenario, config,
                                                     sim_trajectories,
                                                     constraints, car2time2events_updated)
            break
            if has_collision(scenario, sim_trajectories, curves, car_sizes):
                print('Collision in SMT solution. Trying next ASP solution...')
            else:
                print('No collision in SMT solution.')
                break
        except NoSMTSolutionError as err:
            print(err.message)
    if not new_events:
        raise NoSMTSolutionError(
            'No SMT solution found for the ASP solutions!')

    # Update the events
    sim_events.update(new_events)

    return sim_events, curves


def extend(scenario, config):
    import intersection_monitor
    monitor = intersection_monitor.Monitor()

    import scenic
    params = {'map': scenario.map_path,
              'carla_map': scenario.map_name,
              'intersection_uid': scenario.intersection_uid,
              'timestep': scenario.timestep,
              'weather': scenario.weather,
              'render': False,
              'event_monitor': monitor}
    sim_result = {}
    config['ego']['maneuver_uid'] = scenario.maneuver_uid['ego']
    config['ego']['blueprint'] = scenario.blueprints['ego']
    car_sizes = {car: {'width': 0, 'length': 0}
                 for car in config['cars']}  # output parameter of simulation
    # TODO skip simulating ego by using the solvability evidence of the old scenario
    for car in config['cars']:
        print(f'Simulate {car}\'s trajectory...')
        params['car_name'] = car
        params['maneuver_uid'] = config[car]['maneuver_uid']
        params['spawn_distance'] = config[car]['spawn_distance']
        params['car_blueprint'] = config[car]['blueprint']
        params['car_size'] = car_sizes[car]  # output parameter
        scenic_scenario = scenic.scenarioFromFile(
            'trajectory.scenic', params=params)
        scene, _ = scenic_scenario.generate()
        simulator = scenic_scenario.getSimulator()
        settings = simulator.world.get_settings()
        settings.no_rendering_mode = True
        simulator.world.apply_settings(settings)
        sim_result[car] = simulator.simulate(scene, maxSteps=scenario.maxSteps)
        del scenic_scenario, scene

    # Find a strict extension of the given scenario
    sim_trajectories = {}
    for car in config['cars']:
        sim_trajectories[car] = [state[car]
                                 for state in sim_result[car].trajectory]
    sim_trajectories['illegal'] = sim_trajectories['ego']

    new_events, new_curves = solution(
        scenario,
        config,
        monitor.events,
        sim_trajectories,
        car_sizes)

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
        scenario.blueprints, **{car: config[car]['blueprint'] for car in config['cars']})
    scenario_ext.car_sizes = dict(scenario.car_sizes, **car_sizes)
    scenario_ext.maneuver_uid = dict(
        scenario.maneuver_uid, **{car: config[car]['maneuver_uid'] for car in config['cars']})
    scenario_ext.events = dict(scenario.events, **new_events)
    scenario_ext.curves = dict(scenario.curves, **new_curves)
    scenario_ext.sim_trajectories = dict(
        scenario.sim_trajectories, **sim_trajectories)

    return scenario_ext


def new(config):
    from scenario import Scenario
    scenario = Scenario()
    scenario.maxSteps = config['maxSteps']
    scenario.timestep = config['timestep']
    scenario.weather = config['weather']
    scenario.map_path = config['map_path']
    scenario.map_name = config['map_name']
    scenario.intersection_uid = config['intersection_uid']
    scenario.rules_path = config['rules_path']
    scenario.blueprints = {car: config[car]
                           ['blueprint'] for car in config['cars']}
    scenario.car_sizes = {}
    scenario.maneuver_uid = {
        car: config[car]['maneuver_uid'] for car in config['cars']}
    scenario.events = {}
    scenario.curves = {}
    scenario.sim_trajectories = {}

    return extend(scenario, config)
