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


def draw_intersection(world, intersection, draw_lanes=False, arrival_distance=4):
    polygon = intersection.polygon

    # Boundaries of the intersection
    locs = [carla.Location(p[0], -p[1], 0.5) for p in polygon.exterior.coords]
    for i in range(len(locs)):
        p0 = locs[i]
        p1 = locs[(i+1) % len(locs)]
        world.debug.draw_line(
            p0, p1, color=carla.Color(0, 0, 255), life_time=0)

    # Draw lane names
    for lane in intersection.incomingLanes:
        c = lane.centerline[-1]
        v = lane.flowFrom(c, -1)
        loc = carla.Location(v.x, -v.y, 0.5)
        world.debug.draw_string(
            loc, lane.uid, draw_shadow=False, life_time=1000)
    for lane in intersection.outgoingLanes:
        c = lane.centerline[0]
        v = lane.flowFrom(c, 1)
        loc = carla.Location(v.x, -v.y, 0.5)
        world.debug.draw_string(
            loc, lane.uid, draw_shadow=False, life_time=1000)

    # Draw arrival boxes
    for lane in intersection.incomingLanes:
        l = lane.leftEdge[-1]
        vl = lane.flowFrom(l, -arrival_distance)
        r = lane.rightEdge[-1]
        vr = lane.flowFrom(r, -arrival_distance)
        loc_l = carla.Location(vl.x, -vl.y, 0.5)
        loc_r = carla.Location(vr.x, -vr.y, 0.5)
        world.debug.draw_line(
            loc_l, loc_r, thickness=0.1, life_time=1000)

    # Bird-eye view of the intersection
    centroid = polygon.centroid  # a Shapely point
    loc = carla.Location(centroid.x, -centroid.y, 35)
    rot = carla.Rotation(pitch=-90)
    world.get_spectator().set_transform(carla.Transform(loc, rot))

    if draw_lanes:
        for m in intersection.maneuvers:
            l = m.connectingLane
            draw_lane(world, l)


def label_car(world, car):
    loc = carla.Location(car.position.x, -car.position.y, 1.5)
    world.debug.draw_string(loc, car.name, life_time=0.01)


def draw_trajectories(world, sim_trajectory):
    for states in sim_trajectory:
        for state in states.values():
            position = state[0]
            loc = carla.Location(position.x, -position.y, 0.1)
            world.debug.draw_point(loc)
