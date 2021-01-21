#!/home/ak/Scenic/.venv/bin/python

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
