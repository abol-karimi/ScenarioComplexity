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
parser.add_argument('-e', '--ego_from_to', nargs='+', type=str,
                    help='the maneuver of the ego through the intersection')
parser.add_argument('-n', '--nonego_from_to', nargs='+', type=str,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-s', '--nonego_spawn_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
parser.add_argument('-c', '--constraints', nargs='+', type=str, default=[],
                    help='additional logic constraints')
args = parser.parse_args()

scenario = Scenario()

if args.intersection_uid:
    scenario.intersection_uid = args.intersection_uid
if args.rules_path:
    scenario.rules_path = args.rules_path
if args.ego_from_to or args.nonego_from_to:
    from scenic.domains.driving.roads import Network
    network = Network.fromFile(scenario.map_path)
if args.ego_from_to:
    l0_uid = args.ego_from_to[0]
    l2_uid = args.ego_from_to[1]
    for m in network.elements[l0_uid].maneuvers:
        if m.endLane.uid == l2_uid:
            l1_uid = m.connectingLane.uid
    scenario.maneuver_uid['ego'] = (l0_uid, l1_uid, l2_uid)
if args.nonego_from_to:
    l0_uid = args.nonego_from_to[0]
    l2_uid = args.nonego_from_to[1]
    for m in network.elements[l0_uid].maneuvers:
        if m.endLane.uid == l2_uid:
            l1_uid = m.connectingLane.uid
    nonego_maneuver_uid = (l0_uid, l1_uid, l2_uid)

scenario = generator.extend(
    scenario,
    nonego_maneuver_uid=nonego_maneuver_uid,
    nonego_spawn_distance=args.nonego_spawn_distance,
    extra_constraints=args.constraints)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
