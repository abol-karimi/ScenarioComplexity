import enum
from scenic.domains.driving.roads import ManeuverType
import carla
from signals import SignalType


class CarState():
    def __init__(self):
        self.arrived = False
        self.entered = False
        self.exited = False
        self.lanes = set()


def realtime_to_ruletime(T):
    import z3
    if isinstance(T, z3.z3.ArithRef):
        return z3.ToInt(T*2)
    else:
        return int(T*2)


class Event():
    """Abstract class for traffic monitor events."""

    def __init__(self, timestamp, vehicle):
        self.timestamp = timestamp
        self.vehicle = vehicle

    @property
    def ruletime(self):
        return realtime_to_ruletime(self.timestamp.elapsed_seconds)


class ArrivedAtIntersectionEvent(Event):
    """Arrival of a vehicle at an intersection."""
    name = 'arrivedAtForkAtTime'  # TODO declare as a property

    def __init__(self, timestamp, vehicle, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {timeVar})'


class SignaledAtForkEvent(Event):
    """Using a turn signal when arriving at an intersection."""
    name = 'signaledAtForkAtTime'

    def __init__(self, timestamp, vehicle, signal, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.signal = signal
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal.name.lower()}, {self.incoming_lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal.name.lower()}, {self.incoming_lane.uid}, {timeVar})'


class EnteredLaneEvent(Event):
    """When part of a vehicle enters the lane."""
    name = 'enteredLaneAtTime'

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane

    def __str__(self):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane.uid}, {timeVar})'


class ExitedLaneEvent(Event):
    """When the last part of a vehicle exits the lane."""
    name = 'leftLaneAtTime'

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane

    def __str__(self):
        return f'leftLaneAtTime({self.vehicle}, {self.lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'leftLaneAtTime({self.vehicle}, {self.lane.uid}, {timeVar})'


class EnteredIntersectionEvent(Event):
    """When any part of a vehicle enters the intersection."""
    name = 'enteredForkAtTime'

    def __init__(self, timestamp, vehicle, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {timeVar})'


class ExitedIntersectionEvent(Event):
    """When the last part of a vehicle exits the intersection."""
    name = 'exitedFromAtTime'

    def __init__(self, timestamp, vehicle, outgoing_lane):
        super().__init__(timestamp, vehicle)
        self.outgoing_lane = outgoing_lane

    def __str__(self):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane.uid}, {self.ruletime})'

    def withTimeVar(self, timeVar):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane.uid}, {timeVar})'


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    intersection = None
    geometry = []
    events = {}
    nonego = None
    max_realtime = None

    def set_intersection(self, intersection):
        self.intersection = intersection
        self.geometry = []
        self.events = {}
        for maneuver in intersection.maneuvers:
            lane = maneuver.connectingLane
            fork = maneuver.startLane
            exit = maneuver.endLane
            self.geometry.append(
                f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')
            signal = SignalType.from_maneuver(maneuver).name.lower()
            self.geometry.append(
                f'laneCorrectSignal({lane.uid}, {signal})')

        for maneuver in intersection.maneuvers:
            for conflict in maneuver.conflictingManeuvers:
                self.geometry.append(
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
            self.geometry += [
                f'isOnRightOf({right}, {left})' for left in lefts for right in rights]

    def on_arrival(self, timestamp, vehicle, incoming_lane, signal):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(ArrivedAtIntersectionEvent(
            timestamp, vehicle, incoming_lane))
        self.events[vehicle.name].append(SignaledAtForkEvent(
            timestamp, vehicle, signal, incoming_lane))

    def on_enterLane(self, timestamp, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(
            EnteredLaneEvent(timestamp, vehicle, lane))

    def on_exitLane(self, timestamp, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(
            ExitedLaneEvent(timestamp, vehicle, lane))

    def on_entrance(self, timestamp, vehicle, incoming_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(EnteredIntersectionEvent(
            timestamp, vehicle, incoming_lane))

    def on_exit(self, timestamp, vehicle, outgoing_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(ExitedIntersectionEvent(
            timestamp, vehicle, outgoing_lane))

    def nonego_logical_solution(self):
        # Index nonego's events by their ruletime
        time2events = {}
        for event in self.events[self.nonego]:
            if not (event.ruletime in time2events):
                time2events[event.ruletime] = [event]
            else:
                time2events[event.ruletime].append(event)

        # Distinct time variables for nonego's events
        nonego_events = self.events[self.nonego]
        timeVar = {nonego_events[i]
            : f'T{i}' for i in range(len(nonego_events))}

        # Nonego's atoms
        atoms = []

        # Nonego's simultaneous events
        for time in time2events.keys():
            events = time2events[time]
            for i in range(len(events)-1):
                ti = timeVar[events[i]]
                tii = timeVar[events[i+1]]
                atoms.append(
                    f':- {events[i].withTimeVar(ti)}, {events[i+1].withTimeVar(tii)}, {ti} != {tii}')

        # Nonego's non-simultaneous events
        times = sorted(time2events.keys())
        for i in range(len(times)-1):
            ei = time2events[times[i]][0]
            eii = time2events[times[i+1]][0]
            ti = timeVar[ei]
            tii = timeVar[eii]
            atoms += [f':- {ei.withTimeVar(ti)}, {eii.withTimeVar(tii)}, {ti} >= {tii}']

        # Generate nonego events
        for event in self.events[self.nonego]:
            t = timeVar[event]
            atoms += [f'{{ {event.withTimeVar(t)} : time({t}) }} = 1']

        # Instantiate nonego's time variables such that
        #  ego violates nonego's right-of-way.
        from solver import Solver
        max_ruletime = realtime_to_ruletime(self.max_realtime)
        solver = Solver(max_ruletime)
        solver.load('uncontrolled-4way.lp')
        solver.add_atoms(self.geometry)
        for car in self.events.keys():
            if car != self.nonego:
                solver.add_atoms(self.events[car])

        # Nonego atoms
        solver.add_atoms(atoms)

        # Enforce ego's violation of nonego
        solver.add_atoms([f':- not violatesRightOf(ego, {self.nonego})'])

        return solver.solve()

    def nonego_solution(self, sim_result):
        model = self.nonego_logical_solution()

        event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                       'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

        timeless2event = {event.withTimeVar(
            ''): event for event in self.events[self.nonego]}

        ruletime2events = {}
        for atom in model:
            name = atom.name
            args = atom.arguments
            if not (str(args[0]) == self.nonego and name in event_names):
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

        trajectory = sim_result.trajectory
        frame2distance = [0]*len(trajectory)

        for event in self.events[self.nonego]:
            print(f'{event} at frame {event.timestamp.frame}')

        for i in range(len(trajectory)-1):
            pi = trajectory[i][self.nonego][0]
            pii = trajectory[i+1][self.nonego][0]
            frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

        ruletime2distances = {}
        for ruletime, events in ruletime2events.items():
            distances = [frame2distance[event.timestamp.frame]
                         for event in events]
            distances_sorted = sorted(set(distances))
            ruletime2distances[ruletime] = distances_sorted

        import z3
        ruletime2realtimes = {}
        for ruletime, distances in ruletime2distances.items():
            ruletime2realtimes[ruletime] = [
                z3.Real(f'T{ruletime}_{i}') for i in range(len(distances))]

        # Z3 constraints
        constraints = []

        # Valid concretization
        valid_concretization = []

        ruletimes = sorted(ruletime2realtimes.keys())
        realtimes = [realtime for ruletime in ruletimes
                     for realtime in ruletime2realtimes[ruletime]]
        # Shorter distances have earlier realtimes
        valid_concretization += [realtimes[i] < realtimes[i+1]
                                 for i in range(len(realtimes)-1)]
        # Concretizations of a ruletime preserves the ruletime
        for ruletime, realtimes in ruletime2realtimes.items():
            for i in range(len(realtimes)-1):
                Ti = realtime_to_ruletime(realtimes[i])
                Tii = realtime_to_ruletime(realtimes[i+1])
                valid_concretization += [Ti == Tii]

        # Order of ruletimes is preserved by the concretization
        for i in range(len(ruletimes)-1):
            ti = ruletimes[i]
            tii = ruletimes[i+1]
            Ti = realtime_to_ruletime(ruletime2realtimes[ti][0])
            Tii = realtime_to_ruletime(ruletime2realtimes[tii][0])
            valid_concretization += [Ti < Tii]

        constraints += valid_concretization

        # Preserve logical solution (ego violating nonego's right-of-way)
        logical_sol = []
        max_ruletime = realtime_to_ruletime(self.max_realtime)
        ego_ruletimes = sorted(set(
            [event.ruletime for event in self.events['ego']]))
        import portion as P
        ego_intervals = [P.closedopen(0, ego_ruletimes[0])]
        for i in range(len(ego_ruletimes)-1):
            ti = ego_ruletimes[i]
            tii = ego_ruletimes[i+1]
            ego_intervals.append(P.singleton(ti))
            ego_intervals.append(P.open(ti, tii))
        ego_intervals.append(P.singleton(ego_ruletimes[-1]))
        ego_intervals.append(P.openclosed(ego_ruletimes[-1], max_ruletime))

        for t in ruletimes:
            for I in ego_intervals:
                if t in I:
                    T = realtime_to_ruletime(ruletime2realtimes[t][0])
                    if I.lower == I.upper:
                        logical_sol += [T == t]
                    elif I.left is P.OPEN and I.right is P.OPEN:
                        logical_sol += [I.lower < T, T < I.upper]
                    elif I.left is P.CLOSED and I.right is P.OPEN:
                        logical_sol += [I.lower <= T, T < I.upper]
                    elif I.left is P.OPEN and I.right is P.CLOSED:
                        logical_sol += [I.lower < T, T <= I.upper]
                    break

        constraints += logical_sol

        s = z3.Solver()
        s.add(constraints)
        print(s.check())

        # To convert Z3 rational numbers to Python's floating point reals
        def rat2fp(num):
            return float(num.numerator_as_long())/float(num.denominator_as_long())
        m = s.model()
        concrete_realtimes = [rat2fp(m.eval(T)) for T in realtimes]
        print(concrete_realtimes)


monitor = Monitor()
