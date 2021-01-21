#!/home/ak/Scenic/.venv/bin/python
import sys
import getopt


def main(argv):
    inputfile = ''
    try:
        opts, _ = getopt.getopt(argv, "hi:", ["ifile="])
    except getopt.GetoptError:
        print('extend.py -i <inputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('extend.py -i <inputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg

    import pickle
    import scenic

    with open(inputfile, 'rb') as inFile:
        scenario = pickle.load(inFile)

    params = {'map': scenario.map_path,
              'carla_map': scenario.map_name,
              'intersection_id': scenario.intersection_id,
              'maneuver_id': scenario.maneuver_id,
              'timestep': scenario.timestep,
              'weather': scenario.weather,
              'render': True}

    print('Play the loaded scenario...')
    params['trajectory'] = scenario.trajectory
    params['blueprints'] = scenario.blueprints
    params['vehicleLightStates'] = scenario.vehicleLightStates
    scenic_scenario = scenic.scenarioFromFile(
        'replay.scenic', params=params)
    scene, _ = scenic_scenario.generate()
    simulator = scenic_scenario.getSimulator()
    simulator.simulate(scene, maxSteps=scenario.maxSteps)


if __name__ == "__main__":
    main(sys.argv[1:])
