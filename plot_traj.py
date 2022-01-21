#!/usr/bin/env python
from generator import frame_to_realtime
import matplotlib.pyplot as plt
import argparse
import pickle
from geomdl import BSpline
from mscatter import mscatter

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
    t = [frame_to_realtime(f, scenario.timestep)
         for f in range(scenario.maxSteps+1)]
    d = frame2distance[car]
    axs[j].plot(t, d)
    points = curves[car]['ctrlpts']
    t = [p[0] for p in points]
    d = [p[1] for p in points]
    c = ['r' if i % 3 == 0 else 'c' for i in range(len(points))]
    s = [20 if i % 3 == 0 else 10 for i in range(len(points))]
    m = ['d' if i % 3 == 0 else 'o' for i in range(len(points))]
    mscatter(t, d, c=c, s=s, m=m, ax=axs[j])
plt.show()
