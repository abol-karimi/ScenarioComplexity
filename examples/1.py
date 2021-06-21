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
config['map_path'] = './maps/Town05.xodr'
config['map_name'] = 'Town05'
config['intersection_uid'] = 'intersection245'
config['rules_path'] = '4way-stopOnAll.lp'
config['max_speed'] = 8  # m/s
config['constraints'] = []  # ASP statements
config['cars'] = ['ego', 'car1', 'car2', 'car3', 'car4']

config['ego'] = {}
config['ego']['blueprint'] = car_blueprints[2]
config['ego']['maneuver_uid'] = (
    'road9_lane2', 'road304_lane0', 'road45_lane1')
config['ego']['spawn_distance'] = 25

config['car1'] = {}
config['car1']['blueprint'] = car_blueprints[5]
config['car1']['maneuver_uid'] = (
    'road45_lane3', 'road297_lane0', 'road9_lane0')
config['car1']['spawn_distance'] = 10

config['car2'] = {}
config['car2']['blueprint'] = car_blueprints[9]
config['car2']['maneuver_uid'] = (
    'road9_lane3', 'road296_lane0', 'road44_lane3')
config['car2']['spawn_distance'] = 30

config['car3'] = {}
config['car3']['blueprint'] = car_blueprints[16]
config['car3']['maneuver_uid'] = (
    'road44_lane1', 'road291_lane0', 'road9_lane1')
config['car3']['spawn_distance'] = 30

config['car4'] = {}
config['car4']['blueprint'] = car_blueprints[18]
config['car4']['maneuver_uid'] = (
    'road8_lane1', 'road279_lane0', 'road44_lane2')
config['car4']['spawn_distance'] = 30
