import sys
import getopt


def main(argv):
    outputfile = ''
    try:
        opts, _ = getopt.getopt(argv, "ho:", ["ofile="])
    except getopt.GetoptError:
        print('new.py -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('new.py -o <outputfile>')
            sys.exit()
        elif opt in ("-o", "--ofile"):
            outputfile = arg

    from scenario import Scenario
    from generator import Generator

    scenario = Scenario()
    generator = Generator(map_path=scenario.map_path,
                          intersection_id=scenario.intersection_id,
                          timestep=scenario.timestep,
                          maxSteps=scenario.maxSteps)
    scenario = generator.extend(scenario)

    import pickle
    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
