from scenic.domains.driving.roads import Network
from complexgen.core.connecting_lane import connecting_lane as cl

car_blueprints = [
    'vehicle.audi.a2',  # No lights
    'vehicle.audi.etron',  # No lights
    'vehicle.audi.tt',
    'vehicle.bmw.grandtourer',  # No lights
    'vehicle.bmw.isetta',  # No lights
    'vehicle.chevrolet.impala',
    'vehicle.citroen.c3',  # No lights
    'vehicle.dodge_charger.police',
    'vehicle.jeep.wrangler_rubicon',  # No lights
    'vehicle.lincoln.mkz2017',
    'vehicle.mercedes-benz.coupe',  # No lights
    'vehicle.mini.cooperst',  # No lights
    'vehicle.mustang.mustang',  # No lights
    'vehicle.nissan.micra',  # No lights
    'vehicle.nissan.patrol',  # No lights
    'vehicle.seat.leon',  # No lights
    'vehicle.tesla.model3',
    'vehicle.toyota.prius',  # No lights
    'vehicle.volkswagen.t2'
]

night = {'cloudiness': 0.0,
         'precipitation': 0.0,
         'precipitation_deposits': 0.0,
         'wind_intensity': 50.0,
         'sun_azimuth_angle': 0.0,
         'sun_altitude_angle': -90.0,
         'fog_density': 0.0,
         'fog_distance': 0.0,
         'fog_falloff': 0.0,
         'wetness': 0.0
         }

config = {}
config['maxSteps'] = 700
config['timestep'] = 0.05
config['weather'] = 'CloudySunset'
config['map_path'] = '/home/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr'
config['map_name'] = 'Town05'
config['intersection_uid'] = 'intersection224'
config['rules_path'] = '3way-T_stopOnMinor.lp'
config['constraints'] = []  # ASP statements
config['cars'] = ['ego', 'car4']

config['ego'] = {}
config['ego']['blueprint'] = car_blueprints[2]
config['ego']['from_to'] = ('road20_lane1', 'road48_lane1')
config['ego']['spawn_distance'] = 25
config['ego']['maxSpeed'] = 8  # m/s
config['illegal'] = config['ego']

config['car4'] = {}
config['car4']['blueprint'] = 'vehicle.mercedes-benz.coupe'
config['car4']['from_to'] = ('road19_lane0', 'road20_lane0')
config['car4']['spawn_distance'] = 80
config['car4']['maxSpeed'] = 3  # m/s


network = Network.fromFile(config['map_path'])
for car in config['cars']:
    l0 = config[car]['from_to'][0]
    l2 = config[car]['from_to'][1]
    l1 = cl(network, l0, l2)
    config[car]['maneuver_uid'] = (l0, l1, l2)
