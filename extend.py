#!/home/ak/Scenic/.venv/bin/python

import sys
import getopt


def main(argv):
    inputfile = ''
    outputfile = ''
    maneuver_id = None
    try:
        opts, _ = getopt.getopt(
            argv, "hi:o:m:", ["ifile=", "ofile=", "maneuver_id="])
    except getopt.GetoptError:
        print('extend.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('extend.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
        elif opt in ("-m", "--maneuver_id"):
            maneuver_id = int(arg)

    import pickle
    import scenic
    from generator import Generator

    with open(inputfile, 'rb') as inFile:
        scenario = pickle.load(inFile)

    generator = Generator(map_path=scenario.map_path,
                          intersection_id=scenario.intersection_id,
                          timestep=scenario.timestep,
                          maxSteps=scenario.maxSteps)

    scenario = generator.extend(scenario, maneuver_id=maneuver_id)

    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
