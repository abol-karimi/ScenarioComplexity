import sys
import getopt


def main(argv):
    inputfile = ''
    outputfile = ''
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["ifile=", "ofile="])
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

    import pickle
    import scenic
    from generator import Generator

    with open(inputfile, 'rb') as inFile:
        scenario = pickle.load(inFile)

    generator = Generator(map_path=scenario.map_path,
                          intersection_id=scenario.intersection_id,
                          timestep=scenario.timestep,
                          maxSteps=scenario.maxSteps)

    scenario = generator.extend(scenario)

    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
