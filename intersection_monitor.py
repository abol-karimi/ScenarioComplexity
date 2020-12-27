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


class ArrivedAtIntersectionEvent(Event):
    """Arrival of a vehicle at an intersection."""

    def __init__(self, timestamp, vehicle, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {self.timestamp})'


class SignaledAtForkEvent(Event):
    """Using a turn signal when arriving at an intersection."""

    def __init__(self, timestamp, vehicle, signal, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.signal = signal
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal.name.lower()}, {self.incoming_lane.uid}, {self.timestamp})'


class EnteredLaneEvent(Event):
    """When part of a vehicle enters the lane."""

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane

    def __str__(self):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane.uid}, {self.timestamp})'


class ExitedLaneEvent(Event):
    """When the last part of a vehicle exits the lane."""

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane

    def __str__(self):
        return f'leftLaneAtTime({self.vehicle}, {self.lane.uid}, {self.timestamp})'


class EnteredIntersectionEvent(Event):
    """When any part of a vehicle enters the intersection."""

    def __init__(self, timestamp, vehicle, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {self.timestamp})'


class ExitedIntersectionEvent(Event):
    """When the last part of a vehicle exits the intersection."""

    def __init__(self, timestamp, vehicle, outgoing_lane):
        super().__init__(timestamp, vehicle)
        self.outgoing_lane = outgoing_lane

    def __str__(self):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane.uid}, {self.timestamp})'


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

    def timestamp_to_ruletime(self, timestamp):
        return int(timestamp.elapsed_seconds*2)

    def on_arrival(self, timestamp, vehicle, incoming_lane, signal):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        ruletime = self.timestamp_to_ruletime(timestamp)
        self.events[vehicle.name].append(ArrivedAtIntersectionEvent(
            ruletime, vehicle, incoming_lane))
        self.events[vehicle.name].append(SignaledAtForkEvent(
            ruletime, vehicle, signal, incoming_lane))

    def on_enterLane(self, timestamp, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        ruletime = self.timestamp_to_ruletime(timestamp)
        self.events[vehicle.name].append(
            EnteredLaneEvent(ruletime, vehicle, lane))

    def on_exitLane(self, timestamp, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        ruletime = self.timestamp_to_ruletime(timestamp)
        self.events[vehicle.name].append(
            ExitedLaneEvent(ruletime, vehicle, lane))

    def on_entrance(self, timestamp, vehicle, incoming_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        ruletime = self.timestamp_to_ruletime(timestamp)
        self.events[vehicle.name].append(EnteredIntersectionEvent(
            ruletime, vehicle, incoming_lane))

    def on_exit(self, timestamp, vehicle, outgoing_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        ruletime = self.timestamp_to_ruletime(timestamp)
        self.events[vehicle.name].append(ExitedIntersectionEvent(
            ruletime, vehicle, outgoing_lane))

    def violatesRightOf(self, name1, name2):
        from solver import Solver
        solver = Solver("uncontrolled-4way.lp")
        solver.add_atoms(self.geometry)
        solver.add_atoms([event for car in self.events.keys()
                          for event in self.events[car]])
        violations = solver.solve()
        return (tuple([name1, name2]) in violations)

    def nonego_solution(self):
        # Index events by their timestamps
        time2events = {}
        for event in self.events[self.nonego]:
            print(event)
            if not (event.timestamp in time2events):
                time2events[event.timestamp] = [event]
            else:
                time2events[event.timestamp].append(event)

        # Distinct time variables for nonego's events
        events = self.events[self.nonego]
        for i in range(len(events)):
            events[i].timestamp = f'T{i}'

        # Nonego's atoms
        atoms = []

        # Simultaneous events
        for time in time2events.keys():
            events = time2events[time]
            for i in range(len(events)-1):
                ti = events[i].timestamp
                tii = events[i+1].timestamp
                atoms += [f':- {events[i]}, {events[i+1]}, {ti} != {tii}']

        # Non-simultaneous events
        times = sorted(time2events.keys())
        for i in range(len(times)-1):
            ei = time2events[times[i]][0]
            eii = time2events[times[i+1]][0]
            ti = ei.timestamp
            tii = eii.timestamp
            atoms += [f':- {ei}, {eii}, {ti} >= {tii}']

        # Generate events
        for event in self.events[self.nonego]:
            atoms += [f'{{ {event} : time({event.timestamp}) }} = 1']

        from solver import Solver
        solver = Solver("uncontrolled-4way.lp")
        solver.add_atoms(self.geometry)
        for car in self.events.keys():
            if car != self.nonego:
                solver.add_atoms(self.events[car])

        # Nonego atoms
        solver.add_atoms(atoms)

        # Enforce ego's violation of nonego
        solver.add_atoms([f':- not violatesRightOf(ego, {self.nonego})'])

        model = solver.solve()

        event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                       'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
        for atom in model:
            if atom.name in event_names:
                print(atom)


monitor = Monitor()
