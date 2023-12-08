import numpy as np
from scenic.simulators.carla.utils.utils import scenicToCarlaLocation
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