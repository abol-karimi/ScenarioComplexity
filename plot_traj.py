#!/home/ak/Scenic/.venv/bin/python
from generator import frame_to_distance
import matplotlib.pyplot as plt
import argparse
import pickle

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
frame2distance = frame_to_distance(trajectory, car)

plt.title(f'frame-distance curve for {car}')
plt.plot(frame2distance)
plt.scatter([e.frame for e in events],
            [frame2distance[e.frame] for e in events], c='r')
plt.show()
