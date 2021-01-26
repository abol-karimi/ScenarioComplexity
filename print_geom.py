#!/home/ak/Scenic/.venv/bin/python

from generator import load_geometry

map_path = './maps/Town05.xodr'
intersection_id = 3  # unsignalized four-way intersection in Town05
geometry = load_geometry(map_path, intersection_id)

for atom in geometry:
    print(atom)
