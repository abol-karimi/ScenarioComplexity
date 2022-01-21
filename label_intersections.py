#!/usr/bin/env python
import argparse
from scenic.domains.driving.roads import Network
import carla

parser = argparse.ArgumentParser(description='label the intersections.')
parser.add_argument('-m', '--map_name',
                    help='Carla map name', default='Town05')
args = parser.parse_args()

map_name = args.map_name
map_path = f'./maps/{map_name}.xodr'

client = carla.Client('127.0.0.1', 2000)
world = client.load_world(map_name)

network = Network.fromFile(map_path)
for i in network.intersections:
    centroid = i.polygon.centroid
    loc = carla.Location(centroid.x, -centroid.y, 0.5)
    world.debug.draw_string(loc, i.uid, life_time=1000)
