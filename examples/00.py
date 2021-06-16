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
config['max_speed'] = 16  # m/s
config['constraints'] = []  # ASP statements
config['cars'] = ['ego', 'car4']

config['ego'] = {}
config['ego']['spawn_distance'] = 25

config['car4'] = {}
config['car4']['blueprint'] = 'vehicle.bmw.isetta'
config['car4']['maneuver_uid'] = (
    'road8_lane1', 'road259_lane1', 'road9_lane1')
config['car4']['spawn_distance'] = 30
