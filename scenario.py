class Scenario():
    maxSteps = 700
    timestep = 0.05
    weather = 'ClearSunset'
    map_path = './maps/Town05.xodr'
    map_name = 'Town05'
    intersection_id = 3  # unsignalized four-way intersection in Town05
    rules_path = 'uncontrolled-4way.lp'
    maneuver_id = {'ego': 3}
    blueprints = {'ego': 'vehicle.tesla.model3'}
    vehicleLightStates = {}
    trajectory = None
    events = {}
