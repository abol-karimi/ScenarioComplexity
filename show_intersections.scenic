param map = localPath('./maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param render = False

intersections = network.intersections
T_intersections = [i for i in intersections 
  if network.elements[i.uid].is3Way and not network.elements[i.uid].isSignalized]

import visualization


behavior IterateIntersectionsBehavior():
  carla_world = simulation().world
  for intersection in T_intersections:
    visualization.draw_intersection(carla_world, intersection)
    print(f'Intersection uid: {intersection.uid}')
    do FollowLaneBehavior() for 50 seconds
    wait

behavior ShowIntersectionBehavior(uid):
  carla_world = simulation().world
  visualization.draw_intersection(carla_world, network.elements[uid])
  wait

ego = Car with name 'ego',
  with behavior ShowIntersectionBehavior('intersection224')
