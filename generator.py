class Generator():
    """Extend a scenario to a strictly harder one."""

    def __init__(self, map_path=None, intersection_id=None, events={}, nonego=None, timestep=None, maxSteps=None):
        self.geometry = self.load_geometry(map_path, intersection_id)
        self.events = events
        self.nonego = nonego
        self.timestep = timestep
        self.maxSteps = maxSteps
        self.maxSpeed = 8

    def load_geometry(self, map_path, intersection_id):
        from signals import SignalType
        from scenic.domains.driving.roads import Network
        network = Network.fromFile(map_path)
        intersection = network.intersections[intersection_id]
        geometry = []
        self.events = {}
        for maneuver in intersection.maneuvers:
            lane = maneuver.connectingLane
            fork = maneuver.startLane
            exit = maneuver.endLane
            geometry.append(
                f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')
            signal = SignalType.from_maneuver(maneuver).name.lower()
            geometry.append(
                f'laneCorrectSignal({lane.uid}, {signal})')

        for maneuver in intersection.maneuvers:
            for conflict in maneuver.conflictingManeuvers:
                geometry.append(
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
            geometry += [
                f'isOnRightOf({right}, {left})' for left in lefts for right in rights]
        return geometry

    def realtime_to_ruletime(self, t):
        return int(t*2)

    def ruletime_to_realtime(self, T):
        return T/2

    def realtime_to_frame(self, t):
        return int(t/self.timestep)

    def frame_to_realtime(self, frame):
        return frame*self.timestep

    def ruletime_to_frame(self, T):
        realtime = self.ruletime_to_realtime(T)
        return self.realtime_to_frame(realtime)

    def frame_to_ruletime(self, frame):
        realtime = self.frame_to_realtime(frame)
        return self.realtime_to_ruletime(realtime)

    def events_to_trajectory(self, ruletime2events, car, trajectory, frame2distance, ruletime2distances):
        # Interpolate car's trajectory based on
        #  its new (frame, distance) points:
        p_f = [0]
        p_d = [0]
        ruletimes = sorted(ruletime2distances.keys())  # for np.interp()
        for ruletime in ruletimes:
            ds = ruletime2distances[ruletime]
            for d in ds:
                fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
                frame = self.ruletime_to_frame(ruletime + fraction)
                p_f += [frame]
                p_d += [d]
        p_f += [len(frame2distance)-1]
        p_d += [frame2distance[-1]]

        # Linearly interpolate the (frame, distance) points
        import numpy as np
        new2distance = np.interp(range(len(trajectory)), p_f, p_d)

        import matplotlib.pyplot as plt
        plt.plot(frame2distance, 'g')
        plt.plot(p_f, p_d, 'ro')
        plt.plot(new2distance)

        plt.show()

        # The new trajectory
        new2old = []
        old = 0
        for new in range(len(trajectory)):
            while old < len(trajectory)-1 and frame2distance[old] < new2distance[new]:
                old += 1
            new2old += [old]

        new_traj = []
        for frame in range(len(trajectory)):
            new_traj += [trajectory[new2old[frame]][car]]

        return new_traj

    def traj_constraints(self, frame2distance, car):
        maxSpeed = self.maxSpeed

        # Index car's events by their ruletime
        frame2events = {}
        for event in self.events[car]:
            if not (event.frame in frame2events):
                frame2events[event.frame] = [event]
            else:
                frame2events[event.frame].append(event)

        # Constrainst atoms
        atoms = []

        # Car's simultaneous events
        for frame in frame2events.keys():
            events = frame2events[frame]
            for i in range(len(events)-1):
                atoms.append(
                    f':- {events[i].withTime("T1")}, {events[i+1].withTime("T2")}, T1 != T2')

        # Car's non-simultaneous events
        # Two non-simultaneous events may have the same ruletime (logical time)
        frames = sorted(frame2events.keys())
        for i in range(len(frames)-1):
            ei = frame2events[frames[i]][0]
            eii = frame2events[frames[i+1]][0]
            atoms += [f':- {ei.withTime("T1")}, {eii.withTime("T2")}, T1 > T2']

        # Car's speed is bounded between any two events
        for i in range(len(frames)):
            ei = frame2events[frames[i]][0]
            di = frame2distance[frames[i]]
            # Average speed from begining to ti
            atoms += [
                f':- {ei.withTime("T")}, {int(2*di/maxSpeed)+1} > T']
            dinf = frame2distance[-1]
            tinf = self.frame_to_ruletime(self.maxSteps)
            # Average speed from ti to the end
            atoms += [
                f':- {ei.withTime("T")}, {int(2*(dinf-di)/maxSpeed)+1} > {tinf} - T']
            # Average speed from ti to later events
            for j in range(i+1, len(frames)):
                ej = frame2events[frames[j]][0]
                dj = frame2distance[frames[j]]
                delta = int(2*(dj-di)/maxSpeed)+1
                # 0 < delta <= tj - ti
                atoms += [
                    f':- {ei.withTime("T1")}, {ej.withTime("T2")}, {delta} > T2 - T1']

        # Generate car events
        for event in self.events[car]:
            atoms += [f'{{ {event.withTime("T")} : time(T) }} = 1']

        return atoms

    def frame_to_distance(self, sim, car):
        trajectory = sim.trajectory
        frame2distance = [0]*len(trajectory)

        for i in range(len(trajectory)-1):
            pi = trajectory[i][car][0]
            pii = trajectory[i+1][car][0]
            frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

        return frame2distance

    def nocollision(self, car1, car2):
        atoms = []
        # They don't enter the overlap at the same time
        atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
                  f'overlaps(L1, L2),'
                  f'enteredLaneAtTime({car1}, L2, T), leftLaneAtTime({car2}, L1, T)']
        # If car2 enters the overlap first, it exits it before car1 enters it.
        atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
                  f'overlaps(L1, L2),'
                  f'enteredLaneAtTime({car1}, L2, T),'
                  f'enteredLaneByTime({car2}, L1, T), not leftLaneByTime({car2}, L1, T)']
        # If car1 enters the overlap first, it exits it before car2 enters it.
        atoms += [f':- requestedLane({car1}, L1), requestedLane({car2}, L2),'
                  f'overlaps(L1, L2),'
                  f'enteredLaneAtTime({car2}, L1, T),'
                  f'enteredLaneByTime({car1}, L2, T), not leftLaneByTime({car1}, L2, T)']

        return atoms

    def logical_solution(self, frame2distance_ego, frame2distance_illegal, frame2distance_nonego):
        atoms = []
        atoms += self.geometry

        new_cars = {'ego', 'illegal', self.nonego}
        for car in self.events.keys():
            if not car in new_cars:
                atoms += [event.withTime(self.frame_to_ruletime(event.frame))
                          for event in self.events[car]]

        atoms += self.traj_constraints(frame2distance_ego, 'ego')
        atoms += self.traj_constraints(frame2distance_illegal, 'illegal')
        atoms += self.traj_constraints(frame2distance_nonego, self.nonego)

        # No collision
        atoms += self.nocollision('ego', self.nonego)
        old_cars = {car for car in self.events.keys() if car not in new_cars}
        for car in old_cars:
            atoms += self.nocollision(car, 'ego')
            atoms += self.nocollision(car, self.nonego)

        # Enforce ego's legal behavior
        atoms += [f':- violatesRightOf(ego, _)']

        # Evidence that new scenario is strictly harder
        atoms += [f':- not violatesRightOf(illegal, {self.nonego})']

        from solver import Solver
        max_ruletime = self.frame_to_ruletime(self.maxSteps)
        solver = Solver(max_ruletime)
        solver.load('uncontrolled-4way.lp')
        solver.add_atoms(atoms)

        print('Solving...')
        model = solver.solve()

        sol_names = {'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                     'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
        print("Logical solution: ")
        for atom in model:
            if atom.name in sol_names:
                print(f'\t{atom}')

        ruletime2events_ego = self.model_to_events(model, 'ego')
        ruletime2events_nonego = self.model_to_events(model, self.nonego)

        return ruletime2events_ego, ruletime2events_nonego

    def model_to_events(self, model, car):
        event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                       'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

        # To connect logic solution with events
        timeless2event = {event.withTime(
            ''): event for event in self.events[car]}

        # A mapping from new ruletimes to old events
        ruletime2events = {}
        for atom in model:
            name = atom.name
            args = atom.arguments
            if not (str(args[0]) == car and name in event_names):
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

        return ruletime2events

    def solution(self, sim_prev, sim_ego, sim_nonego):
        import copy
        self.events['illegal'] = []
        for event in self.events['ego']:
            event_ill = copy.copy(event)
            event_ill.vehicle = 'illegal'
            self.events['illegal'] += [event_ill]

        frame2distance_ego = self.frame_to_distance(sim_ego, 'ego')
        frame2distance_illegal = self.frame_to_distance(
            sim_ego, 'ego')  # TODO can use frame2distance_ego
        frame2distance_nonego = self.frame_to_distance(sim_nonego, self.nonego)

        ruletime2events_ego, ruletime2events_nonego = self.logical_solution(frame2distance_ego,
                                                                            frame2distance_illegal, frame2distance_nonego)

        # Distances of events of a ruletime in increasing order
        ruletime2distances_ego = {}
        for ruletime, events in ruletime2events_ego.items():
            distances = [frame2distance_ego[event.frame]
                         for event in events]
            distances_sorted = sorted(set(distances))
            ruletime2distances_ego[ruletime] = distances_sorted

        # Distances of events of a ruletime in increasing order
        ruletime2distances_nonego = {}
        for ruletime, events in ruletime2events_nonego.items():
            distances = [frame2distance_nonego[event.frame]
                         for event in events]
            distances_sorted = sorted(set(distances))
            ruletime2distances_nonego[ruletime] = distances_sorted

        trajectory_ego = sim_ego.trajectory
        trajectory_nonego = sim_nonego.trajectory

        # Interpolate the events to a new trajectory
        new_traj_ego = self.events_to_trajectory(
            ruletime2events_ego, 'ego', trajectory_ego, frame2distance_ego, ruletime2distances_ego)
        new_traj_nonego = self.events_to_trajectory(
            ruletime2events_nonego, self.nonego, trajectory_nonego, frame2distance_nonego, ruletime2distances_nonego)

        # The new scenario
        if sim_prev:
            sim_result = sim_prev
        else:
            from scenic.core.simulators import SimulationResult
            sim_result = SimulationResult(
                [{} for i in range(len(trajectory_ego))], [], 'Harder scenario found.')
        trajectory = sim_result.trajectory

        # Trajectories of the new vehicles
        for frame in range(len(trajectory)):
            trajectory[frame]['ego'] = new_traj_ego[frame]
            trajectory[frame][self.nonego] = new_traj_nonego[frame]

        # Update timing of new cars' events
        for ruletime, events in ruletime2events_ego.items():
            ds = ruletime2distances_ego[ruletime]
            for event in events:
                d = frame2distance_ego[event.frame]
                fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
                frame = self.ruletime_to_frame(ruletime + fraction)
                event.frame = frame
        for ruletime, events in ruletime2events_nonego.items():
            ds = ruletime2distances_nonego[ruletime]
            for event in events:
                d = frame2distance_nonego[event.frame]
                fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
                frame = self.ruletime_to_frame(ruletime + fraction)
                event.frame = frame

        return sim_result
