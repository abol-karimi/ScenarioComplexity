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


class Event():
    """Abstract class for traffic monitor events."""

    def __init__(self, timestamp, vehicle):
        self.timestamp = timestamp
        self.vehicle = vehicle

    @property
    def ruletime(self):
        return int(self.timestamp.elapsed_seconds*2)


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

    def nonego_solution(self, sim_result):
        # Index nonego's events by their timestamps
        time2events = {}
        for event in self.events[self.nonego]:
            print(event)
            if not (event.ruletime in time2events):
                time2events[event.ruletime] = [event]
            else:
                time2events[event.ruletime].append(event)

        # Distinct time variables for nonego's events
        events = self.events[self.nonego]
        timeVar = {events[i]: f'T{i}' for i in range(len(events))}

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
        solver = Solver()
        solver.load('uncontrolled-4way.lp')
        solver.add_atoms(self.geometry)
        for car in self.events.keys():
            if car != self.nonego:
                solver.add_atoms(self.events[car])

        # Nonego atoms
        solver.add_atoms(atoms)

        # Enforce ego's violation of nonego
        solver.add_atoms([f':- not violatesRightOf(ego, {self.nonego})'])

        model = solver.solve()

        trajectory = sim_result.trajectory
        distances = [0]*len(trajectory)
        for i in range(len(trajectory)-1):
            pi = trajectory[i][self.nonego][0]
            pii = trajectory[i+1][self.nonego][0]
            distances[i+1] = distances[i] + pi.distanceTo(pii)

        for event in self.events[self.nonego]:
            print(event)
            print(event.withTimeVar(timeVar[event]))

        # Remember only the order of nonego events relative to ego's
        # import portion as P
        # ego_times = sorted([event.ruletime for event in self.events['ego']])
        # ego_intervals = []
        # for i in range(len(ego_times)):


monitor = Monitor()
