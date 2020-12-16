import enum
from scenic.domains.driving.roads import ManeuverType
import carla


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


@enum.unique
class SignalType(enum.Enum):
    """Turn signal at an intersection."""
    OFF = enum.auto()
    LEFT = enum.auto()
    RIGHT = enum.auto()

    @classmethod
    def from_maneuver(cls, maneuver):
        if maneuver.type is ManeuverType.STRAIGHT:
            return SignalType.OFF
        if maneuver.type is ManeuverType.LEFT_TURN:
            return SignalType.LEFT
        if maneuver.type is ManeuverType.RIGHT_TURN:
            return SignalType.RIGHT
        if maneuver.type is ManeuverType.U_TURN:
            return SignalType.LEFT


class SignaledAtIntersectionEvent(Event):
    """Using a turn signal when arriving at an intersection."""

    def __init__(self, timestamp, vehicle, signal, incoming_lane):
        super().__init__(timestamp, vehicle)
        self.signal = signal
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal}, {self.incoming_lane}, {self.timestamp})'


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


class ExitedIntersectionEvent(Event):
    """When the last part of a vehicle exits the intersection."""

    def __init__(self, timestamp, vehicle, outgoing_lane):
        super().__init__(timestamp, vehicle)
        self.outgoing_lane = outgoing_lane


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    intersection = None
    geometry = []
    events = []

    def set_intersection(self, intersection):
        self.intersection = intersection
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

    def on_arrival(self, timestamp, vehicle, incoming_lane):
        self.events.append(ArrivedAtIntersectionEvent(
            timestamp, vehicle, incoming_lane))

    def on_enterLane(self, timestamp, vehicle, lane):
        self.events.append(EnteredLaneEvent(timestamp, vehicle, lane))

    def on_exitLane(self, timestamp, vehicle, lane):
        self.events.append(ExitedLaneEvent(timestamp, vehicle, lane))


monitor = Monitor()
