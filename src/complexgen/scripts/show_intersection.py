#!/usr/bin/env python3.8
import argparse
from scenic.domains.driving.roads import Network
import carla
from complexgen.simulators.carla.visualization import draw_intersection

parser = argparse.ArgumentParser(
    description='Show a bird-eye view of the intersection.')
parser.add_argument('map_name', help='Carla map name')
parser.add_argument('intersection_uid', help='Scenic uid for the intersection')
parser.add_argument('--draw_lanes', action='store_true',
                    help='Visualize lane boundaries')

args = parser.parse_args()

map_name = args.map_name
map_path = f'/home/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/{map_name}.xodr'

client = carla.Client('127.0.0.1', 2000)
world = client.load_world(map_name)

network = Network.fromFile(map_path)
intersection = network.elements[args.intersection_uid]
draw_intersection(world, intersection, draw_lanes=args.draw_lanes)
