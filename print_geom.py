#!/usr/bin/env python3.8

from scenic.domains.driving.roads import Network
from generator import geometry_atoms
import argparse
import carla

parser = argparse.ArgumentParser(description='label the intersections.')
parser.add_argument('map_name', help='Carla map name')
parser.add_argument('intersection_uid', help='Carla map name')
args = parser.parse_args()

map_name = args.map_name
map_path = f'./maps/{map_name}.xodr'

client = carla.Client('127.0.0.1', 2000)
world = client.load_world(map_name)

network = Network.fromFile(map_path)
geometry = geometry_atoms(network, args.intersection_uid)

for atom in geometry:
    print(atom)
