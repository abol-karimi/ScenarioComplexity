from geomdl import BSpline
import numpy as np
import math
from scenic.simulators.carla.utils.utils import scenicToCarlaLocation
from scenic.core.object_types import Point
from scenic.core.regions import RectangularRegion
import carla
from visualization import draw_rect
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    raise RuntimeError(
        'cannot import PIL, make sure "Pillow" package is installed')

from matplotlib import cm
VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


def draw_names(cars, image, camera):
    # Build the K projection matrix:
    # K = [[Fx,  0, image_w/2],
    #      [ 0, Fy, image_h/2],
    #      [ 0,  0,         1]]
    image_w = int(camera.attributes['image_size_x'])
    image_h = int(camera.attributes['image_size_y'])
    fov = float(camera.attributes['fov'])
    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

    # In this case Fx and Fy are the same since the pixel aspect
    # ratio is 1
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0

    # Get the raw BGRA buffer and convert it to an array of RGB of
    # shape (image.height, image.width, 3).
    im_array = np.copy(np.frombuffer(
        image.raw_data, dtype=np.dtype("uint8")))
    im_array = np.reshape(
        im_array, (image.height, image.width, 4))
    im_array = im_array[:, :, :3][:, :, ::-1]

    # Get the lidar data and convert it to a numpy array.
    locations = [scenicToCarlaLocation(car.position, z=1) for car in cars]
    world_points = np.array([[l.x, l.y, l.z, 1] for l in locations]).T

    # This (4, 4) matrix transforms the points from world to sensor coordinates.
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # Transform the points from world space to camera space.
    sensor_points = np.dot(world_2_camera, world_points)

    # New we must change from UE4's coordinate system to an "standard"
    # camera coordinate system (the same used by OpenCV):

    # ^ z                       . z
    # |                        /
    # |              to:      +-------> x
    # | . x                   |
    # |/                      |
    # +-------> y             v y

    # This can be achieved by multiplying by the following matrix:
    # [[ 0,  1,  0 ],
    #  [ 0,  0, -1 ],
    #  [ 1,  0,  0 ]]

    # Or, in this case, is the same as swapping:
    # (x, y ,z) -> (y, -z, x)
    point_in_camera_coords = np.array([
        sensor_points[1],
        sensor_points[2] * -1,
        sensor_points[0]])

    # Finally we can use our K matrix to do the actual 3D -> 2D.
    points_2d = np.dot(K, point_in_camera_coords)

    # Remember to normalize the x, y values by the 3rd value.
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]])

    # At this point, points_2d[0, :] contains all the x and points_2d[1, :]
    # contains all the y values of our points. In order to properly
    # visualize everything on a screen, the points that are out of the screen
    # must be discarted, the same with points behind the camera projection plane.
    points_2d = points_2d.T
    points_in_canvas_mask = \
        (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < image_w) & \
        (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < image_h) & \
        (points_2d[:, 2] > 0.0)
    points_2d = points_2d[points_in_canvas_mask]

    # Extract the screen coords (uv) as integers.
    u_coord = points_2d[:, 0].astype(np.int)
    v_coord = points_2d[:, 1].astype(np.int)

    # The given image
    out = Image.fromarray(im_array)
    # get a drawing context
    d = ImageDraw.Draw(out)
    # get a font
    fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 40)

    # TODO not all cars may be in camera's view
    for i in range(len(points_2d)):
        d.text((u_coord[i], v_coord[i]),
               cars[i].name,
               font=fnt, anchor='mm', stroke_width=1, fill=(255, 255, 255))

    # Return a Pillow image
    return out


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

    import numpy as np
    pi = np.pi
    xs_i = np.interp(distances, ds, xs)
    ys_i = np.interp(distances, ds, ys)
    hs_i = np.interp(distances, ds, np.unwrap(hs))
    # wrap headings back to (-pi,pi):
    hs_i = [(h + pi) % (2*pi) - pi for h in hs_i]

    from scenic.core.vectors import Vector
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
