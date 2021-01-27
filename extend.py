#!/home/ak/Scenic/.venv/bin/python
import generator
import pickle
import argparse


parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
parser.add_argument('outputfile', help='filename of the new scenario')
parser.add_argument('-m', '--from_to', nargs='+', type=str,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-s', '--nonego_spawn_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

if args.from_to:
    from scenic.domains.driving.roads import Network
    network = Network.fromFile(scenario.map_path)
    l0_uid = args.from_to[0]
    l2_uid = args.from_to[1]
    for m in network.elements[l0_uid].maneuvers:
        if m.endLane.uid == l2_uid:
            l1_uid = m.connectingLane.uid
    nonego_maneuver_uid = (l0_uid, l1_uid, l2_uid)
else:
    nonego_maneuver_uid = ('road9_lane2', 'road304_lane0', 'road45_lane1')


scenario = generator.extend(
    scenario,
    nonego_maneuver_uid=nonego_maneuver_uid,
    nonego_spawn_distance=args.nonego_spawn_distance)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
