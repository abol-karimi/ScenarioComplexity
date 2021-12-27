class Scenario():
    maxSteps = None
    timestep = None
    weather = None
    map_path = None
    map_name = None
    intersection_uid = None
    rules_path = None
    blueprints = None
    car_sizes = None
    maneuver_uid = None
    events = None
    # time-to-distance functions (determines the speed profile of each car):
    curves = None  # TODO rename
    # pose sequence (interpolating points of distance-to-pose functions):
    sim_trajectories = None  # TODO rename
