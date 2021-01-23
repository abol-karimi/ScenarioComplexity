param map = localPath('./maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param render = False

intersections = network.intersections
ints = {i: intersections[i] for i in range(len(intersections))
  if intersections[i].is3Way and not intersections[i].isSignalized}
import visualization


behavior IterateIntersectionsBehavior():
  carla_world = simulation().world
  for i, intersection in ints.items():
    visualization.draw_intersection(carla_world, intersection)
    print(f'Intersection index: {i}')
    do FollowLaneBehavior() for 50 seconds
    wait

behavior ShowIntersectionBehavior(id):
  carla_world = simulation().world
  visualization.draw_intersection(carla_world, intersections[id])
  wait

ego = Car with name 'ego',
  with behavior ShowIntersectionBehavior(18)
