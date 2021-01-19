class Event():
    """Abstract class for traffic monitor events."""

    def __init__(self, frame, vehicle):
        self.frame = frame
        self.vehicle = vehicle


class ArrivedAtIntersectionEvent(Event):
    """Arrival of a vehicle at an intersection."""
    name = 'arrivedAtForkAtTime'  # TODO declare as a property

    def __init__(self, frame, vehicle, incoming_lane):
        super().__init__(frame, vehicle)
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'arrivedAtForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {t})'


class SignaledAtForkEvent(Event):
    """Using a turn signal when arriving at an intersection."""
    name = 'signaledAtForkAtTime'

    def __init__(self, frame, vehicle, signal, incoming_lane):
        super().__init__(frame, vehicle)
        self.signal = signal
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'signaledAtForkAtTime({self.vehicle}, {self.signal.name.lower()}, {self.incoming_lane.uid}, {t})'


class EnteredLaneEvent(Event):
    """When part of a vehicle enters the lane."""
    name = 'enteredLaneAtTime'

    def __init__(self, frame, vehicle, lane):
        super().__init__(frame, vehicle)
        self.lane = lane

    def withTime(self, t):
        return f'enteredLaneAtTime({self.vehicle}, {self.lane.uid}, {t})'


class ExitedLaneEvent(Event):
    """When the last part of a vehicle exits the lane."""
    name = 'leftLaneAtTime'

    def __init__(self, frame, vehicle, lane):
        super().__init__(frame, vehicle)
        self.lane = lane

    def withTime(self, t):
        return f'leftLaneAtTime({self.vehicle}, {self.lane.uid}, {t})'


class EnteredIntersectionEvent(Event):
    """When any part of a vehicle enters the intersection."""
    name = 'enteredForkAtTime'

    def __init__(self, frame, vehicle, incoming_lane):
        super().__init__(frame, vehicle)
        self.incoming_lane = incoming_lane

    def withTime(self, t):
        return f'enteredForkAtTime({self.vehicle}, {self.incoming_lane.uid}, {t})'


class ExitedIntersectionEvent(Event):
    """When the last part of a vehicle exits the intersection."""
    name = 'exitedFromAtTime'

    def __init__(self, frame, vehicle, outgoing_lane):
        super().__init__(frame, vehicle)
        self.outgoing_lane = outgoing_lane

    def withTime(self, t):
        return f'exitedFromAtTime({self.vehicle}, {self.outgoing_lane.uid}, {t})'


class Monitor():
    """Record all the static and dynamic traffic predicates."""
    events = {}

    def on_arrival(self, frame, vehicle, incoming_lane, signal):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(ArrivedAtIntersectionEvent(
            frame, vehicle, incoming_lane))
        self.events[vehicle.name].append(SignaledAtForkEvent(
            frame, vehicle, signal, incoming_lane))

    def on_enterLane(self, frame, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(
            EnteredLaneEvent(frame, vehicle, lane))

    def on_exitLane(self, frame, vehicle, lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(
            ExitedLaneEvent(frame, vehicle, lane))

    def on_entrance(self, frame, vehicle, incoming_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(EnteredIntersectionEvent(
            frame, vehicle, incoming_lane))

    def on_exit(self, frame, vehicle, outgoing_lane):
        if not (vehicle.name in self.events):
            self.events[vehicle.name] = []
        self.events[vehicle.name].append(ExitedIntersectionEvent(
            frame, vehicle, outgoing_lane))
