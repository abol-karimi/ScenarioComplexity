#!/home/ak/Scenic/.venv/bin/python
import pickle
import generator
from scenario import Scenario
import argparse


parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('outputfile', help='filename of the new scenario')
parser.add_argument('-i', '--intersection_uid', type=str,
                    help='the intersection at which the scenario happens')
parser.add_argument('-r', '--rules_path',
                    help='the file containing the traffic rules')
parser.add_argument('-e', '--ego_maneuver_uid', nargs='+', type=str,
                    help='the maneuver of the ego through the intersection')
parser.add_argument('-n', '--nonego_maneuver_uid', nargs='+', type=str,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-s', '--nonego_spawn_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
args = parser.parse_args()

scenario = Scenario()
if args.intersection_uid:
    scenario.intersection_uid = args.intersection_uid
if args.rules_path:
    scenario.rules_path = args.rules_path
if args.ego_maneuver_uid:
    scenario.maneuver_uid['ego'] = tuple(args.ego_maneuver_uid)

scenario = generator.extend(
    scenario,
    nonego_maneuver_uid=tuple(args.nonego_maneuver_uid),
    nonego_spawn_distance=args.nonego_spawn_distance)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
