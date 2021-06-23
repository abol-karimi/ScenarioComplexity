#!/home/ak/Scenic/.venv/bin/python
from generator import realtime_to_frame_float
import matplotlib.pyplot as plt
import argparse
import pickle
from geomdl import BSpline

parser = argparse.ArgumentParser(
    description='Plot frame-distance curve of a car.')
parser.add_argument('inputfile', help='filename of the given scenario')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

curves = scenario.curves
frame2distance = {}
for car, params in curves.items():
    curve = BSpline.Curve()
    curve.degree = params['degree']
    curve.ctrlpts = params['ctrlpts']
    curve.knotvector = params['knotvector']
    curve.sample_size = int(scenario.maxSteps)+1
    frame2distance[car] = [p[1] for p in curve.evalpts]

fig, axs = plt.subplots(len(curves))
fig.suptitle('time-distance curves')
for j, car in enumerate(curves):
    axs[j].set_title(car)
    axs[j].plot(frame2distance[car])
    points = curves[car]['ctrlpts']
    t = [realtime_to_frame_float(p[0], scenario.timestep) for p in points]
    d = [p[1] for p in points]
    axs[j].scatter(t, d,
                   c=['r' if i % 3 == 0 else 'b' for i in range(len(points))],
                   s=[10 if i % 3 == 0 else 5 for i in range(len(points))])
plt.show()
