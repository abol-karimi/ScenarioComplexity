#!/home/ak/Scenic/.venv/bin/python

import carla

client = carla.Client('127.0.0.1', 2000)
world = client.get_world()
blueprint_library = world.get_blueprint_library()
for v in blueprint_library.filter('vehicle.*.*'):
    print(v.id)
