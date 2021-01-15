class Generator():
    """Extend a scenario to a strictly harder one."""

    def __init__(self, map_path=None, intersection_id=None, events={}, nonego=None, timestep=None, maxSteps=None):
        self.geometry = self.load_geometry(map_path, intersection_id)
        self.events = events
        self.nonego = nonego
        self.timestep = timestep
        self.maxSteps = maxSteps

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

    def nonego_logical_solution(self, frame2distance):
        # Index nonego's events by their ruletime
        frame2events = {}
        for event in self.events[self.nonego]:
            if not (event.frame in frame2events):
                frame2events[event.frame] = [event]
            else:
                frame2events[event.frame].append(event)

        # Distinct time variables for nonego's events
        nonego_events = self.events[self.nonego]
        timeVar = {nonego_events[i]                   : f'T{i}' for i in range(len(nonego_events))}

        # Nonego's atoms
        atoms = []

        # Nonego's simultaneous events
        for frame in frame2events.keys():
            events = frame2events[frame]
            for i in range(len(events)-1):
                ti = timeVar[events[i]]
                tii = timeVar[events[i+1]]
                atoms.append(
                    f':- {events[i].withTime(ti)}, {events[i+1].withTime(tii)}, {ti} != {tii}')

        # Nonego's non-simultaneous events
        # An event with earlier frame cannot have a later ruletime
        frames = sorted(frame2events.keys())
        for i in range(len(frames)-1):
            ei = frame2events[frames[i]][0]
            eii = frame2events[frames[i+1]][0]
            ti = timeVar[ei]
            tii = timeVar[eii]
            atoms += [f':- {ei.withTime(ti)}, {eii.withTime(tii)}, {ti} > {tii}']

        # Nonego's speed is bounded between any two events
        max_speed = 7  # m/s
        for i in range(len(frames)):
            ei = frame2events[frames[i]][0]
            di = frame2distance[frames[i]]
            ti = timeVar[ei]
            # Average speed from begining to ti
            atoms += [
                f':- {ei.withTime(ti)}, {int(2*di/max_speed)+1} > {ti}']
            dinf = frame2distance[-1]
            tinf = self.frame_to_ruletime(self.maxSteps)
            # Average speed from ti to the end
            atoms += [
                f':- {ei.withTime(ti)}, {int(2*(dinf-di)/max_speed)+1} > {tinf} - {ti}']
            # Average speed from ti to later events
            for j in range(i+1, len(frames)):
                ej = frame2events[frames[j]][0]
                dj = frame2distance[frames[j]]
                tj = timeVar[ej]
                delta = int(2*(dj-di)/max_speed)+1
                # 0 < delta <= tj - ti
                atoms += [
                    f':- {ei.withTime(ti)}, {ej.withTime(tj)}, {delta} > {tj} - {ti}']

        # Generate nonego events
        for event in self.events[self.nonego]:
            t = timeVar[event]
            atoms += [f'{{ {event.withTime(t)} : time({t}) }} = 1']

        # Instantiate nonego's time variables such that
        #  ego violates nonego's right-of-way.
        from solver import Solver
        max_ruletime = self.frame_to_ruletime(self.maxSteps)
        solver = Solver(max_ruletime)
        solver.load('uncontrolled-4way.lp')
        solver.add_atoms(self.geometry)

        # Ego atoms
        solver.add_atoms([event.withTime(self.frame_to_ruletime(
            event.frame)) for event in self.events['ego']])

        # Nonego atoms
        solver.add_atoms(atoms)

        # Enforce ego's violation of nonego
        solver.add_atoms([f':- not violatesRightOf(ego, {self.nonego})'])

        m = solver.solve()

        sol_names = {'mustYieldToForRuleAtTime', 'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                     'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

        sol_names = sol_names.union({'signaledLeft',
                                     'onSameHighway',
                                     'requestedLane',
                                     'arrivedAtTime'})
        print("Nonego's logical solution: ")
        for atom in m:
            if atom.name in sol_names:
                print(f'\t{atom}')

        return m

    def nonego_solution(self, sim_result):
        trajectory = sim_result.trajectory
        frame2distance = [0]*len(trajectory)

        for i in range(len(trajectory)-1):
            pi = trajectory[i][self.nonego][0]
            pii = trajectory[i+1][self.nonego][0]
            frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

        model = self.nonego_logical_solution(frame2distance)

        event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                       'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

        # To connect logic solution with monitor events
        timeless2event = {event.withTime(
            ''): event for event in self.events[self.nonego]}

        # A mapping from new ruletimes to old events
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

        # Distances of events of a ruletime in increasing order
        ruletime2distances = {}
        for ruletime, events in ruletime2events.items():
            distances = [frame2distance[event.frame]
                         for event in events]
            distances_sorted = sorted(set(distances))
            ruletime2distances[ruletime] = distances_sorted

        # Update timing of nonego's events
        for ruletime, events in ruletime2events.items():
            ds = ruletime2distances[ruletime]
            for event in events:
                d = frame2distance[event.frame]
                fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
                frame = self.ruletime_to_frame(ruletime + fraction)
                event.frame = frame

        # Update the trajectory of nonego.
        # (frame, distance) points for nonego's events:
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

        # Update the trajectory
        new2old = []
        old = 0
        for new in range(len(trajectory)):
            while old < len(trajectory)-1 and frame2distance[old] < new2distance[new]:
                old += 1
            new2old += [old]

        new_traj = []
        for frame in range(len(trajectory)):
            new_traj += [trajectory[new2old[frame]][self.nonego]]
        for frame in range(len(trajectory)):
            trajectory[frame][self.nonego] = new_traj[frame]

    def ego_logical_solution(self, frame2distance):
        # Index ego's events by their ruletime
        frame2events = {}
        for event in self.events['ego']:
            if not (event.frame in frame2events):
                frame2events[event.frame] = [event]
            else:
                frame2events[event.frame].append(event)

        # Distinct time variables for ego's events
        ego_events = self.events['ego']
        timeVar = {ego_events[i]: f'T{i}' for i in range(len(ego_events))}

        # Ego's atoms
        atoms = []

        # Nonego's simultaneous events
        for frame in frame2events.keys():
            events = frame2events[frame]
            for i in range(len(events)-1):
                ti = timeVar[events[i]]
                tii = timeVar[events[i+1]]
                atoms.append(
                    f':- {events[i].withTime(ti)}, {events[i+1].withTime(tii)}, {ti} != {tii}')

        # Ego's non-simultaneous events
        # Two non-simultaneous events may have the same ruletime (logical time)
        frames = sorted(frame2events.keys())
        for i in range(len(frames)-1):
            ei = frame2events[frames[i]][0]
            eii = frame2events[frames[i+1]][0]
            ti = timeVar[ei]
            tii = timeVar[eii]
            atoms += [f':- {ei.withTime(ti)}, {eii.withTime(tii)}, {ti} > {tii}']

        # Ego's speed is bounded between any two events
        max_speed = 7  # m/s
        for i in range(len(frames)):
            ei = frame2events[frames[i]][0]
            di = frame2distance[frames[i]]
            ti = timeVar[ei]
            # Average speed from begining to ti
            atoms += [
                f':- {ei.withTime(ti)}, {int(2*di/max_speed)+1} > {ti}']
            dinf = frame2distance[-1]
            tinf = self.frame_to_ruletime(self.maxSteps)
            # Average speed from ti to the end
            atoms += [
                f':- {ei.withTime(ti)}, {int(2*(dinf-di)/max_speed)+1} > {tinf} - {ti}']
            # Average speed from ti to later events
            for j in range(i+1, len(frames)):
                ej = frame2events[frames[j]][0]
                dj = frame2distance[frames[j]]
                tj = timeVar[ej]
                delta = int(2*(dj-di)/max_speed)+1
                # 0 < delta <= tj - ti
                atoms += [
                    f':- {ei.withTime(ti)}, {ej.withTime(tj)}, {delta} > {tj} - {ti}']

        # Generate ego events
        for event in self.events['ego']:
            t = timeVar[event]
            atoms += [f'{{ {event.withTime(t)} : time({t}) }} = 1']

        # Instantiate ego's time variables such that
        #  ego does not violate any nonego's right-of-way.
        from solver import Solver
        max_ruletime = self.frame_to_ruletime(self.maxSteps)
        solver = Solver(max_ruletime)
        solver.load('uncontrolled-4way.lp')
        solver.add_atoms(self.geometry)
        for car in self.events.keys():
            if car != 'ego':
                solver.add_atoms([event.withTime(self.frame_to_ruletime(
                    event.frame)) for event in self.events[car]])

        # Ego atoms
        solver.add_atoms(atoms)

        # Enforce ego's legal behavior
        solver.add_atoms([f':- violatesRightOf(ego, _)'])

        m = solver.solve()

        sol_names = {'violatesRightOfForRule', 'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                     'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}
        print("Ego's logical solution: ")
        for atom in m:
            if atom.name in sol_names:
                print(f'\t{atom}')

        return m

    def ego_solution(self, sim_result):
        trajectory = sim_result.trajectory
        frame2distance = [0]*len(trajectory)

        for i in range(len(trajectory)-1):
            pi = trajectory[i]['ego'][0]
            pii = trajectory[i+1]['ego'][0]
            frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

        model = self.ego_logical_solution(frame2distance)

        event_names = {'arrivedAtForkAtTime', 'signaledAtForkAtTime',
                       'enteredLaneAtTime', 'leftLaneAtTime', 'enteredForkAtTime', 'exitedFromAtTime'}

        # To connect logic solution with monitor events
        timeless2event = {event.withTime(
            ''): event for event in self.events['ego']}

        # A mapping from new ruletimes to old events
        ruletime2events = {}
        for atom in model:
            name = atom.name
            args = atom.arguments
            if not (str(args[0]) == 'ego' and name in event_names):
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

        # Distances of events of a ruletime in increasing order
        ruletime2distances = {}
        for ruletime, events in ruletime2events.items():
            distances = [frame2distance[event.frame]
                         for event in events]
            distances_sorted = sorted(set(distances))
            ruletime2distances[ruletime] = distances_sorted

        # Update timing of ego's events
        for ruletime, events in ruletime2events.items():
            ds = ruletime2distances[ruletime]
            for event in events:
                d = frame2distance[event.frame]
                fraction = (d-ds[0])/(ds[-1]-ds[0]+1)
                frame = self.ruletime_to_frame(ruletime + fraction)
                event.frame = frame

        # Update the trajectory of ego.
        # (frame, distance) points for ego's events:
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

        # Update the trajectory
        new2old = []
        old = 0
        for new in range(len(trajectory)):
            while old < len(trajectory)-1 and frame2distance[old] < new2distance[new]:
                old += 1
            new2old += [old]

        new_traj = []
        for frame in range(len(trajectory)):
            new_traj += [trajectory[new2old[frame]]['ego']]
        for frame in range(len(trajectory)):
            trajectory[frame]['ego'] = new_traj[frame]
