import enum
from scenic.domains.driving.roads import ManeuverType
import carla
from signals import SignalType


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
    events = []

    def set_intersection(self, intersection):
        self.intersection = intersection
        self.geometry = []
        self.events = []
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
        self.events.append(ArrivedAtIntersectionEvent(
            timestamp, vehicle, incoming_lane))
        self.events.append(SignaledAtForkEvent(
            timestamp, vehicle, signal, incoming_lane))

    def on_enterLane(self, timestamp, vehicle, lane):
        self.events.append(EnteredLaneEvent(timestamp, vehicle, lane))

    def on_exitLane(self, timestamp, vehicle, lane):
        self.events.append(ExitedLaneEvent(timestamp, vehicle, lane))

    def on_entrance(self, timestamp, vehicle, incoming_lane):
        self.events.append(EnteredIntersectionEvent(
            timestamp, vehicle, incoming_lane))

    def on_exit(self, timestamp, vehicle, outgoing_lane):
        self.events.append(ExitedIntersectionEvent(
            timestamp, vehicle, outgoing_lane))


class CarState():
    def __init__(self):
        self.arrived = False
        self.entered = False
        self.exited = False
        self.lanes = set()


monitor = Monitor()
