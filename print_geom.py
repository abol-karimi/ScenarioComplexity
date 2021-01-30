#!/home/ak/Scenic/.venv/bin/python

from scenic.domains.driving.roads import Network
from generator import geometry_atoms

# A city
map_path = './maps/Town05.xodr'
# unsignalized four-way intersection in Town05
intersection_uid = 'intersection245'
# 3way T-intersection with stop sign on minor road in Town05
intersection_uid = 'intersection224'

network = Network.fromFile(map_path)
geometry = geometry_atoms(network, intersection_uid)

for atom in geometry:
    print(atom)
