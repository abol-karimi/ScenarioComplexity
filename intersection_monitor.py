class ArrivedAtIntersectionEvent:
    """Arrival of a vehicle at an intersection."""
    name = 'arrivedAtForkAtTime'  # TODO declare as a property

    def __init__(self, vehicle, incoming_lane, frame):
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane
        self.frame = frame

    def withTime(self, t):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class SignaledAtForkEvent:
    """Using a turn signal when arriving at an intersection."""
    name = 'signaledAtForkAtTime'

    def __init__(self, vehicle, signal, incoming_lane, frame):
        self.vehicle = vehicle
        self.signal = signal
        self.incoming_lane = incoming_lane
        self.frame = frame

    def withTime(self, t):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal}, {self.incoming_lane}, {t})'


class StoppedAtForkEvent:
    """Stopping after arrival at a stop sign and before entrance to the intersection."""
    name = 'stoppedAtForkAtTime'

    def __init__(self, vehicle, incoming_lane, frame):
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane
        self.frame = frame

    def withTime(self, t):
        return f'stoppedAtForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class EnteredLaneEvent:
    """When part of a vehicle enters the lane."""
    name = 'enteredLaneAtTime'

    def __init__(self, vehicle, lane, frame):
        self.vehicle = vehicle
        self.lane = lane
        self.frame = frame

    def withTime(self, t):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane}, {t})'


class ExitedLaneEvent:
    """When the last part of a vehicle exits the lane."""
    name = 'leftLaneAtTime'

    def __init__(self, vehicle, lane, frame):
        self.vehicle = vehicle
        self.lane = lane
        self.frame = frame

    def withTime(self, t):
        return f'leftLaneAtTime({self.vehicle}, {self.lane}, {t})'


class EnteredIntersectionEvent:
    """When any part of a vehicle enters the intersection."""
    name = 'enteredForkAtTime'

    def __init__(self, vehicle, incoming_lane, frame):
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane
        self.frame = frame

    def withTime(self, t):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class ExitedIntersectionEvent:
    """When the last part of a vehicle exits the intersection."""
    name = 'exitedFromAtTime'

    def __init__(self, vehicle, outgoing_lane, frame):
        self.vehicle = vehicle
        self.outgoing_lane = outgoing_lane
        self.frame = frame

    def withTime(self, t):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane}, {t})'


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    events = {}

    def on_arrival(self, vehicle, incoming_lane, signal, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(ArrivedAtIntersectionEvent(
            vehicle, incoming_lane, frame))
        self.events[vehicle].append(SignaledAtForkEvent(
            vehicle, signal, incoming_lane, frame))

    def on_stop(self, vehicle, incoming_lane, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            StoppedAtForkEvent(vehicle, incoming_lane, frame))

    def on_enterLane(self, vehicle, lane, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            EnteredLaneEvent(vehicle, lane, frame))

    def on_exitLane(self, vehicle, lane, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            ExitedLaneEvent(vehicle, lane, frame))

    def on_entrance(self, vehicle, incoming_lane, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(EnteredIntersectionEvent(
            vehicle, incoming_lane, frame))

    def on_exit(self, vehicle, outgoing_lane, frame):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(ExitedIntersectionEvent(
            vehicle, outgoing_lane, frame))
