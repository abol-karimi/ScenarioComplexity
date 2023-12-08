#!/usr/bin/env python3.8
import complexgen.core.generator as generator
import jsonpickle
import argparse
import importlib

parser = argparse.ArgumentParser(description='generate a new scenario.')
parser.add_argument('input', help='filename of the given scenario')
parser.add_argument('output', help='filename of the new scenario')
parser.add_argument('config', help='filename of the scenario parameters')
args = parser.parse_args()

with open(args.input, 'r') as f:
    scenario = jsonpickle.decode(f.read())

config_path = args.config.replace('/', '.').replace('.py', '')
config_module = importlib.import_module(config_path, package=None)

scenario = generator.extend(scenario, config_module.config)

with open(args.output, 'w') as f:
    f.write(jsonpickle.encode(scenario))
