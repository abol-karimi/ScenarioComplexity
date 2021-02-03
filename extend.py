#!/home/ak/Scenic/.venv/bin/python
import generator
import pickle
import argparse
import random

car_blueprints = [
    'vehicle.audi.a2',
    'vehicle.audi.etron',
    'vehicle.audi.tt',
    'vehicle.bmw.grandtourer',
    'vehicle.bmw.isetta',
    'vehicle.chevrolet.impala',
    'vehicle.citroen.c3',
    'vehicle.dodge_charger.police',
    'vehicle.jeep.wrangler_rubicon',
    'vehicle.lincoln.mkz2017',
    'vehicle.mercedes-benz.coupe',
    'vehicle.mini.cooperst',
    'vehicle.mustang.mustang',
    'vehicle.nissan.micra',
    'vehicle.nissan.patrol',
    'vehicle.seat.leon',
    'vehicle.tesla.model3',
    'vehicle.toyota.prius',
    'vehicle.volkswagen.t2'
]
nonego_blueprint = random.choice(car_blueprints)

parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('inputfile', help='filename of the given scenario')
parser.add_argument('outputfile', help='filename of the new scenario')
parser.add_argument('-m', '--from_to', nargs='+', type=str,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-d', '--nonego_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
parser.add_argument('-s', '--max_speed', type=float, default=8.0,
                    help='max average speed of new cars')
parser.add_argument('-b', '--blueprint',
                    help='blueprint of the nonego')
parser.add_argument('-c', '--constraints', nargs='+', type=str, default=[],
                    help='additional logic constraints')
args = parser.parse_args()

if args.blueprint:
    nonego_blueprint = args.blueprints

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
    nonego_maneuver_uid,
    nonego_spawn_distance=args.nonego_distance,
    maxSpeed=args.max_speed,
    nonego_blueprint=nonego_blueprint,
    extra_constraints=args.constraints)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
