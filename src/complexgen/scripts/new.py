#!/usr/bin/env python3.8
import jsonpickle
import complexgen.core.generator as generator
import argparse
import importlib

parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('output', help='filename of the new scenario')
parser.add_argument('config', help='filename of the scenario parameters')
args = parser.parse_args()

config_path = args.config.replace('/', '.').replace('.py', '')

config_module = importlib.import_module(config_path, package=None)
scenario = generator.new(config_module.config)

with open(args.output, 'w') as f:
    f.write(jsonpickle.encode(scenario))
