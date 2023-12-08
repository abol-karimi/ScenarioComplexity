from geomdl import BSpline
import numpy as np

from scenic.core.regions import RectangularRegion
from scenic.core.vectors import Vector


def frame_to_distance(trajectory):
    frame2distance = [0]*len(trajectory)

    for i in range(len(trajectory)-1):
        pi = trajectory[i][0]
        pii = trajectory[i+1][0]
        frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

    return frame2distance


def distance_to_pose(distances, sim_distances, traj):
    """ For each frame, we are given a distance in 'sim_distances' and a corresponding pose in 'traj'.
    We return the poses corresponding to 'distances' by linear interpolation of the above (distance, pose) pairs.
    """
    ds, xs, ys, hs = [], [], [], []
    last_dist = -1
    for d, pose in zip(sim_distances, traj):
        if last_dist == d:
            continue
        last_dist = d
        # add data points
        x, y, h = pose[0].x, pose[0].y, pose[1]
        ds.append(d), xs.append(x), ys.append(y), hs.append(h)

    pi = np.pi
    xs_i = np.interp(distances, ds, xs)
    ys_i = np.interp(distances, ds, ys)
    hs_i = np.interp(distances, ds, np.unwrap(hs))
    # wrap headings back to (-pi,pi):
    hs_i = [(h + pi) % (2*pi) - pi for h in hs_i]

    poses = [[Vector(x, y), h] for x, y, h in zip(xs_i, ys_i, hs_i)]
    return poses


def spline_to_traj(degree, ctrlpts, knotvector, sample_size, sim_traj):
    curve = BSpline.Curve()
    curve.degree = degree
    curve.ctrlpts = ctrlpts
    curve.knotvector = knotvector
    curve.sample_size = sample_size
    frame2distance = [p[1] for p in curve.evalpts]
    frame2simDistance = frame_to_distance(sim_traj)
    traj = distance_to_pose(
        frame2distance, frame2simDistance, sim_traj)
    return traj


def curves_to_trajectories(curves, sim_trajs, sample_size):
    new_trajs = {}
    for car, curve in curves.items():
        degree = curve['degree']
        ctrlpts = curve['ctrlpts']
        knotvector = curve['knotvector']
        traj = spline_to_traj(degree, ctrlpts, knotvector,
                              sample_size, sim_trajs[car])
        new_trajs[car] = traj
    return new_trajs


def collision(traj1, size1, traj2, size2):
    w1, l1 = size1['width'], size1['length']
    w2, l2 = size2['width'], size2['length']
    # client = carla.Client('127.0.0.1', 2000)
    # world = client.get_world()
    for i, (pose1, pose2) in enumerate(zip(traj1, traj2)):
        p1, h1 = pose1[0], pose1[1]
        p2, h2 = pose2[0], pose2[1]
        rect1 = RectangularRegion(p1, h1, w1, l1)
        rect2 = RectangularRegion(p2, h2, w2, l2)
        bCollision = rect1.intersects(rect2)
        if bCollision:
            print('Collision at time step {i}')
            # draw_rect(world, rect1, i*.04)
            # draw_rect(world, rect2, i*.04)
            return True
    return False


def has_collision(scenario, new_poses, new_curves, new_sizes):
    sizes = dict(scenario.car_sizes, **new_sizes)
    time_to_dist = dict(scenario.curves, **new_curves)
    poses = dict(scenario.sim_trajectories, **new_poses)
    sample_size = int(scenario.maxSteps)+1
    traj = curves_to_trajectories(time_to_dist, poses, sample_size)

    old_nonegos = [car for car in scenario.events if not car in {
        'ego', 'illegal'}]
    new_nonegos = [car for car in new_sizes if not car in {
        'ego', 'illegal'}]
    print(old_nonegos)
    print(new_nonegos)
    # between new non-egos
    for i, new1 in enumerate(new_nonegos):
        for new2 in new_nonegos[i+1:]:
            if collision(traj[new1], sizes[new1], traj[new2], sizes[new2]):
                print(f'{new1} collides with {new2}')
                return True

    # between new and old non-egos
    for new in new_nonegos:
        for old in old_nonegos:
            if collision(traj[new], sizes[new], traj[old], sizes[old]):
                print(f'{new} collides with {old}')
                return True

    # between ego and old or new nonegos
    for nonego in old_nonegos+new_nonegos:
        if collision(traj['ego'], sizes['ego'], traj[nonego], sizes[nonego]):
            print(f'ego collides with {nonego}')
            return True

    # between illegal and old nonegos
    for old in old_nonegos:
        if collision(traj['illegal'], sizes['ego'], traj[old], sizes[old]):
            print(f'illegal collides with {old}')
            return True

    return False
