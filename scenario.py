class Scenario():
    maxSteps = 700
    timestep = 0.05
    weather = 'ClearSunset'
    map_path = './maps/Town05.xodr'
    map_name = 'Town05'
    # unsignalized four-way intersection in Town05:
    intersection_uid = 'intersection245'
    rules_path = '4way-uncontrolled.lp'
    blueprints = {'ego': 'vehicle.tesla.model3'}
    # left turn from road9_lane2 in Town05:
    maneuver_uid = {'ego': ('road9_lane2', 'road304_lane0', 'road45_lane1')}
    trajectory = None
    events = {}
