""" Scenario Description
Ego-vehicle arrives at an intersection.
"""
param map = localPath('/home/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')  # or other CARLA map that definitely works
param carla_map = 'Town05'
model scenic.simulators.carla.model

param replay_scenario = None
replay_scenario = globalParameters.replay_scenario
intersection = network.elements[replay_scenario.intersection_uid]
maneuver_uid = replay_scenario.maneuver_uid
blueprints = replay_scenario.blueprints
events = replay_scenario.events
curves = replay_scenario.curves
sim_trajs = replay_scenario.sim_trajectories
sample_size = int(replay_scenario.maxSteps)+1
from utils import curves_to_trajectories
trajectory = curves_to_trajectories(curves, sim_trajs, sample_size)

param keyframes = None 
keyframes = globalParameters.keyframes

param images = None
images = globalParameters.images

import visualization
import carla
import utils
from PIL import ImageDraw, ImageFont
from generator import frame_to_realtime

behavior ReplayBehavior():
	carla_world = simulation().world
	while True:
		t = simulation().currentTime
		state = trajectory[self.name][t]
		take SetTransformAction(state[0], state[1])
		visualization.label_car(carla_world, self)

for carName, traj in trajectory.items():
	carState = traj[0]
	if not carName in {'ego', 'illegal'}:
		car = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with color Color(0, 0, 1),
			with behavior ReplayBehavior(),
			with physics False
	elif carName == 'ego':
		ego = Car at carState[0], facing carState[1],
			with name carName,
			with blueprint blueprints[carName],
			with color Color(0, 1, 0),
			with behavior ReplayBehavior(),
			with physics False

illegal = Car ahead of ego by ego.length,
	with name 'illegal',
	with blueprint blueprints['ego'],
	with color Color(1, 0, 0),
	with behavior ReplayBehavior(),
	with physics False

key_events = {f: '' for f in keyframes}
for events in replay_scenario.events.values():
	for e in events:
		if e.frame in keyframes:
				realtime = frame_to_realtime(e.frame, replay_scenario.timestep)
				key_events[e.frame] += e.withTime(realtime) + '\n'

cameras = []

def cam_callback(image):
	cam = cameras[0]
	try:
		t = simulation().currentTime
	except AssertionError:
		cam.destroy()
		return
	if t in keyframes:
		print(f'Captured image at frame {t}')
		cars = [obj for obj in simulation().objects if isinstance(obj, Car)]
		cars = [car for car in cars if (distance from (front of car) to intersection) < 5]
		images[t] = utils.draw_names(cars, image, cam)
		# Draw events
		d = ImageDraw.Draw(images[t])
		fnt = ImageFont.truetype("Pillow/Tests/fonts/FreeMono.ttf", 55)
		d.multiline_text((0,0), key_events[t], font=fnt, stroke_width=3, fill=(255, 0, 0))

def setup_camera():
	carla_world = simulation().world
	centroid = intersection.polygon.centroid  # a Shapely point
	loc = carla.Location(centroid.x, -centroid.y, 16)
	rot = carla.Rotation(pitch=-90)
	cam_transform = carla.Transform(loc, rot)

	cam_bp = None
	cam_bp = carla_world.get_blueprint_library().find('sensor.camera.rgb')
	cam_bp.set_attribute("image_size_x",str(1700))
	cam_bp.set_attribute("image_size_y",str(1800))
	cam_bp.set_attribute("fov",str(90))
	over_cam = carla_world.spawn_actor(cam_bp, cam_transform)
	over_cam.listen(cam_callback)
	cameras.append(over_cam)

monitor showIntersection:
	carla_world = simulation().world
	visualization.draw_intersection(carla_world, intersection, draw_lanes=True)
	setup_camera()
	wait
	