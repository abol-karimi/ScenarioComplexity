#!/home/ak/Scenic/.venv/bin/python

from generator import load_geometry

map_path = './maps/Town05.xodr'
# unsignalized four-way intersection in Town05
intersection_uid = 'intersection245'
geometry = load_geometry(map_path, intersection_uid)

for atom in geometry:
    print(atom)
