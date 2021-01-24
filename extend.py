#!/home/ak/Scenic/.venv/bin/python

import sys
import getopt


def main(argv):
    inputfile = ''
    outputfile = ''
    maneuver_id = None
    nonego_spawn_distance = None
    try:
        opts, _ = getopt.getopt(
            argv, "hi:o:m:s:",
            ["ifile=", "ofile=", "maneuver_id=", "nonego_spawn_distance="])
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
        elif opt in ("-s", "--nonego_spawn_distance"):
            nonego_spawn_distance = float(arg)

    import pickle
    import scenic
    from generator import Generator

    with open(inputfile, 'rb') as inFile:
        scenario = pickle.load(inFile)

    generator = Generator()

    scenario = generator.extend(
        scenario, nonego_maneuver_id=maneuver_id, nonego_spawn_distance=nonego_spawn_distance)

    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
