#!/home/ak/Scenic/.venv/bin/python
import pickle
import generator
from scenario import Scenario
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
parser.add_argument('outputfile', help='filename of the new scenario')
parser.add_argument('-i', '--intersection_uid', type=str,
                    help='the intersection at which the scenario happens')
parser.add_argument('-r', '--rules_path',
                    help='the file containing the traffic rules')
parser.add_argument('-e', '--ego_from_to', nargs='+', type=str,
                    help='the maneuver of the ego through the intersection')
parser.add_argument('-n', '--nonego_from_to', nargs='+', type=str,
                    help='the maneuver of the nonego through the intersection')
parser.add_argument('-s', '--max_speed', type=float, default=8.0,
                    help='max average speed of new cars')
parser.add_argument('--ego_distance', type=float, default=25.0,
                    help='initial distance of ego to the intersection')
parser.add_argument('--nonego_distance', type=float, default=10.0,
                    help='initial distance of nonego to the intersection')
parser.add_argument('--ego_blueprint',
                    help='blueprint of ego')
parser.add_argument('--nonego_blueprint',
                    help='blueprint of the nonego')
parser.add_argument('-c', '--constraints', nargs='+', type=str, default=[],
                    help='additional logic constraints')
args = parser.parse_args()

if args.nonego_blueprint:
    nonego_blueprint = args.blueprints

scenario = Scenario()

if args.ego_blueprint:
    scenario.blueprints['ego'] = args.ego_blueprint
else:
    scenario.blueprints['ego'] = random.choice(car_blueprints)

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
    nonego_maneuver_uid,
    ego_spawn_distance=args.ego_distance,
    nonego_spawn_distance=args.nonego_distance,
    maxSpeed=args.max_speed,
    nonego_blueprint=nonego_blueprint,
    extra_constraints=args.constraints)

with open(args.outputfile, 'wb') as outFile:
    pickle.dump(scenario, outFile)
