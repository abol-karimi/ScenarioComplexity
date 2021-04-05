class ArrivedAtIntersectionEvent:
    """Arrival of a vehicle at an intersection."""
    name = 'arrivedAtForkAtTime'  # TODO declare as a property

    def __init__(self, frame, vehicle, incoming_lane):
        self.frame = frame
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class SignaledAtForkEvent:
    """Using a turn signal when arriving at an intersection."""
    name = 'signaledAtForkAtTime'

    def __init__(self, frame, vehicle, signal, incoming_lane):
        self.frame = frame
        self.vehicle = vehicle
        self.signal = signal
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal.name.lower()}, {self.incoming_lane}, {t})'


class StoppedAtForkEvent:
    """Stopping after arrival at a stop sign and before entrance to the intersection."""
    name = 'stoppedAtForkAtTime'

    def __init__(self, frame, vehicle, incoming_lane):
        self.frame = frame
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'stoppedAtForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class EnteredLaneEvent:
    """When part of a vehicle enters the lane."""
    name = 'enteredLaneAtTime'

    def __init__(self, frame, vehicle, lane):
        self.frame = frame
        self.vehicle = vehicle
        self.lane = lane

    def withTime(self, t):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane}, {t})'


class ExitedLaneEvent:
    """When the last part of a vehicle exits the lane."""
    name = 'leftLaneAtTime'

    def __init__(self, frame, vehicle, lane):
        self.frame = frame
        self.vehicle = vehicle
        self.lane = lane

    def withTime(self, t):
        return f'leftLaneAtTime({self.vehicle}, {self.lane}, {t})'


class EnteredIntersectionEvent:
    """When any part of a vehicle enters the intersection."""
    name = 'enteredForkAtTime'

    def __init__(self, frame, vehicle, incoming_lane):
        self.frame = frame
        self.vehicle = vehicle
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane}, {t})'


class ExitedIntersectionEvent:
    """When the last part of a vehicle exits the intersection."""
    name = 'exitedFromAtTime'

    def __init__(self, frame, vehicle, outgoing_lane):
        self.frame = frame
        self.vehicle = vehicle
        self.outgoing_lane = outgoing_lane

    def withTime(self, t):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane}, {t})'


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    events = {}

    def on_arrival(self, frame, vehicle, incoming_lane, signal):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(ArrivedAtIntersectionEvent(
            frame, vehicle, incoming_lane))
        self.events[vehicle].append(SignaledAtForkEvent(
            frame, vehicle, signal, incoming_lane))

    def on_stop(self, frame, vehicle, incoming_lane):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            StoppedAtForkEvent(frame, vehicle, incoming_lane))

    def on_enterLane(self, frame, vehicle, lane):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            EnteredLaneEvent(frame, vehicle, lane))

    def on_exitLane(self, frame, vehicle, lane):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(
            ExitedLaneEvent(frame, vehicle, lane))

    def on_entrance(self, frame, vehicle, incoming_lane):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(EnteredIntersectionEvent(
            frame, vehicle, incoming_lane))

    def on_exit(self, frame, vehicle, outgoing_lane):
        if not (vehicle in self.events):
            self.events[vehicle] = []
        self.events[vehicle].append(ExitedIntersectionEvent(
            frame, vehicle, outgoing_lane))
