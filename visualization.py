import carla


def draw_lane(world, lane, color=carla.Color(255, 0, 0), life_time=-1):
    locations = [carla.Location(p[0], -p[1], 0.1)
                 for p in lane.leftEdge.lineString.coords]
    for i in range(len(locations)-1):
        begin = locations[i]
        end = locations[i+1]
        world.debug.draw_line(
            begin, end, thickness=0.1, color=color, life_time=life_time)
    locations = [carla.Location(p[0], -p[1], 0.1)
                 for p in lane.rightEdge.lineString.coords]
    for i in range(len(locations)-1):
        begin = locations[i]
        end = locations[i+1]
        world.debug.draw_line(
            begin, end, thickness=0.1, color=color, life_time=life_time)


def draw_intersection(world, intersection):
    polygon = intersection.polygon
    centroid = polygon.centroid
    print(type(centroid))
    loc = carla.Location(centroid.x, -centroid.y, 30)
    rot = carla.Rotation(pitch=-90)
    world.get_spectator().set_transform(carla.Transform(loc, rot))
    print(polygon.centroid)
