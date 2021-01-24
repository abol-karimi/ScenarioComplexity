#!/home/ak/Scenic/.venv/bin/python

from signals import SignalType
from scenic.domains.driving.roads import Network

map_path = './maps/Town05.xodr'
intersection_id = 3  # unsignalized four-way intersection in Town05
network = Network.fromFile(map_path)
intersection = network.intersections[intersection_id]
maneuvers = intersection.maneuvers
for i in range(len(maneuvers)):
    m = maneuvers[i]
    print(f'Maneuver {i}: { m.connectingLane.uid}')
    print(f'  From {m.startLane.uid} to {m.endLane.uid}')


geometry = []
for maneuver in intersection.maneuvers:
    lane = maneuver.connectingLane
    fork = maneuver.startLane
    exit = maneuver.endLane
    geometry.append(
        f'laneFromTo({lane.uid}, {fork.uid}, {exit.uid})')
    signal = SignalType.from_maneuver(maneuver).name.lower()
    geometry.append(
        f'laneCorrectSignal({lane.uid}, {signal})')

for i in range(len(maneuvers)):
    li = maneuvers[i].connectingLane
    for j in range(i+1, len(maneuvers)):
        lj = maneuvers[j].connectingLane
        if li.intersects(lj):
            geometry.append(f'overlaps({li.uid}, {lj.uid})')
            geometry.append(f'overlaps({lj.uid}, {li.uid})')

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

for atom in geometry:
    print(atom)
