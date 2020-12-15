import enum
from scenic.domains.driving.roads import ManeuverType
import carla


class Event():
    """Abstract class for traffic monitor events."""

    def __init__(self, timestamp, actor):
        self.timestamp = timestamp
        self.actor = actor


class ArrivedAtIntersectionEvent(Event):
    """Arrival of a vehicle at an intersection."""

    def __init__(self, timestamp, actor, incoming_lane):
        super().__init__(timestamp, actor)
        self.incoming_lane = incoming_lane


@enum.unique
class SignalType(enum.Enum):
    """Turn signal at an intersection."""
    OFF = enum.auto()
    LEFT = enum.auto()
    RIGHT = enum.auto()

    @classmethod
    def from_maneuver(cls, maneuver):
        if maneuver is ManeuverType.STRAIGHT:
            return SignalType.OFF
        if maneuver is ManeuverType.LEFT_TURN:
            return SignalType.LEFT
        if maneuver is ManeuverType.RIGHT_TURN:
            return SignalType.RIGHT
        if maneuver is ManeuverType.U_TURN:
            return SignalType.LEFT


class SignaledAtIntersectionEvent(Event):
    """Using a turn signal when arriving at an intersection."""

    def __init__(self, timestamp, actor, signal, incoming_lane):
        super().__init__(timestamp, actor)
        self.signal = signal
        self.incoming_lane = incoming_lane

    def __str__(self):
        return f'signaledAtForkAtTime('


class EnteredLaneEvent(Event):
    """When part of a vehicle enters the lane."""

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane


class ExitedLaneEvent(Event):
    """When the last part of a vehicle exits the lane."""

    def __init__(self, timestamp, vehicle, lane):
        super().__init__(timestamp, vehicle)
        self.lane = lane


class ExitedIntersectionEvent(Event):
    """When the last part of a vehicle exits the intersection."""

    def __init__(self, timestamp, vehicle, outgoing_lane):
        super().__init__(timestamp, vehicle)
        self.outgoing_lane = outgoing_lane


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    intersection = None
    geometry = []
    forkId = {}
    exitId = {}
    maneuverId = {}
    laneId = {}
    events = []

    def set_world(self, carla_world):
        self.world = carla_world

    def set_intersection(self, intersection):
        self.intersection = intersection
        forkId = self.forkId = {}
        forks = intersection.incomingLanes
        for index in range(len(forks)):
            forkId[forks[index]] = index

        exitId = self.exitId = {}
        exits = intersection.outgoingLanes
        for index in range(len(exits)):
            exitId[exits[index]] = index

        maneuverId = self.maneuverId = {}
        laneId = self.laneId = {}
        maneuvers = intersection.maneuvers
        for index in range(len(maneuvers)):
            maneuver = maneuvers[index]
            maneuverId[maneuver] = index
            laneId[maneuver.connectingLane] = index
            fork = maneuver.startLane
            exit = maneuver.endLane
            print("Lane from " + "F" + str(forkId[fork]) + " to E" +
                  str(exitId[exit]) + " is " + maneuver.type.name)

        for maneuver in maneuvers:
            for conflict in maneuver.conflictingManeuvers:
                print(
                    "overlaps(" + str(maneuverId[maneuver]) + ", " + str(maneuverId[conflict]) + ")")

    def on_arrival(self):
        print("A vehicle arrived!")

    def draw_lane(self, lane, color=carla.Color(255, 0, 0), life_time=-1):
        world = self.world
        locations = [carla.Location(p[0], -p[1], 0.1)
                     for p in lane.leftEdge.lineString.coords]
        for i in range(len(locations)-1):
            begin = locations[i]
            end = locations[i+1]
            world.debug.draw_line(
                begin, end, thickness=0.1, color=color, life_time=life_time)
        locations = [carla.Location(p[0], -p[1], 0.1)
                     for p in lane.rightEdge.lineString.coords]
        for i in range(len(locations)-1):
            begin = locations[i]
            end = locations[i+1]
            world.debug.draw_line(
                begin, end, thickness=0.1, color=color, life_time=life_time)


monitor = Monitor()
