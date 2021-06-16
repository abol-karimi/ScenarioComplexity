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

config = {}
config['maxSteps'] = 700
config['timestep'] = 0.05
config['weather'] = 'ClearSunset'
config['map_path'] = './maps/Town05.xodr'
config['map_name'] = 'Town05'
config['intersection_uid'] = 'intersection245'
config['rules_path'] = '4way-stopOnAll.lp'
config['max_speed'] = 8  # m/s
config['constraints'] = []  # ASP statements
config['cars'] = ['ego', 'car1', 'car2', 'car3']

config['ego'] = {}
config['ego']['blueprint'] = 'vehicle.tesla.model3'
config['ego']['maneuver_uid'] = (
    'road9_lane2', 'road304_lane0', 'road45_lane1')
config['ego']['spawn_distance'] = 25

config['car1'] = {}
config['car1']['blueprint'] = 'vehicle.volkswagen.t2'
config['car1']['maneuver_uid'] = (
    'road44_lane1', 'road329_lane1', 'road45_lane1')
config['car1']['spawn_distance'] = 10

config['car2'] = {}
config['car2']['blueprint'] = 'vehicle.chevrolet.impala'
config['car2']['maneuver_uid'] = (
    'road44_lane0', 'road275_lane0', 'road8_lane3')
config['car2']['spawn_distance'] = 30

config['car3'] = {}
config['car3']['blueprint'] = 'vehicle.bmw.isetta'
config['car3']['maneuver_uid'] = (
    'road8_lane0', 'road287_lane0', 'road45_lane0')
config['car3']['spawn_distance'] = 30
