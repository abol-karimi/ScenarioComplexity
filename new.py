#!/home/ak/Scenic/.venv/bin/python
import sys
import getopt


def main(argv):
    outputfile = ''
    intersection_id = None
    rules_path = None
    try:
        opts, _ = getopt.getopt(
            argv, "ho:i:r:", ["ofile=", "intersection_id=", "rules_path="])
    except getopt.GetoptError:
        print('new.py -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('new.py -o <outputfile>')
            sys.exit()
        elif opt in ("-o", "--ofile"):
            outputfile = arg
        elif opt in ("-i", "--intersection_id="):
            intersection_id = int(arg)
        elif opt in ("-r", "--rules_path="):
            rules_path = arg

    from scenario import Scenario
    from generator import Generator

    scenario = Scenario()
    if intersection_id:
        scenario.intersection_id = intersection_id
    if rules_path:
        scenario.rules_path = rules_path
    generator = Generator(map_path=scenario.map_path,
                          intersection_id=scenario.intersection_id,
                          rules_path=scenario.rules_path,
                          timestep=scenario.timestep,
                          maxSteps=scenario.maxSteps)
    scenario = generator.extend(scenario)

    import pickle
    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
