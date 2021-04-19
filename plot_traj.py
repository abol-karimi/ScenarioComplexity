#!/home/ak/Scenic/.venv/bin/python
import matplotlib.pyplot as plt
from scenic.domains.driving.roads import Network
from solver import Solver
import argparse
from generator import frame_to_ruletime
import pickle
import scenic
from generator import geometry_atoms, frame_to_ruletime

parser = argparse.ArgumentParser(
    description='Plot frame-distance curve of a car.')
parser.add_argument('inputfile', help='filename of the given scenario')
parser.add_argument('car', help='name of the car')
args = parser.parse_args()

with open(args.inputfile, 'rb') as inFile:
    scenario = pickle.load(inFile)

car = args.car
trajectory = scenario.trajectory
events = scenario.events[car]
# calculate the curve TODO import from generator
frame2distance = [0]*len(trajectory)
for i in range(len(trajectory)-1):
    pi = trajectory[i][car][0]
    pii = trajectory[i+1][car][0]
    frame2distance[i+1] = frame2distance[i] + pi.distanceTo(pii)

plt.title(f'frame-distance curve for {car}')
plt.plot(frame2distance)
plt.scatter([e.frame for e in events],
            [frame2distance[e.frame] for e in events], c='r')
plt.show()
