#!/home/ak/Scenic/.venv/bin/python
import generator
import pickle
import argparse


parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
parser.add_argument('outputfile', help='filename of the new scenario')
parser.add_argument('-m', '--maneuver_id', type=int, default=0,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-s', '--nonego_spawn_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

scenario = generator.extend(
    scenario,
    nonego_maneuver_id=args.maneuver_id,
    nonego_spawn_distance=args.nonego_spawn_distance)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
