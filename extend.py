#!/usr/bin/env python3.8
import generator
import pickle
import argparse
import importlib

parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('input', help='filename of the given scenario')
parser.add_argument('output', help='filename of the new scenario')
parser.add_argument('config', help='filename of the scenario parameters')
args = parser.parse_args()

with open(args.input, 'rb') as inFile:
    scenario = pickle.load(inFile)

config_path = args.config.replace('/', '.').replace('.py', '')
config_module = importlib.import_module(config_path, package=None)

scenario = generator.extend(scenario, config_module.config)

with open(args.output, 'wb') as outFile:
    pickle.dump(scenario, outFile)
