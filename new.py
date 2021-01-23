#!/home/ak/Scenic/.venv/bin/python
import sys
import getopt


def main(argv):
    outputfile = ''
    intersection_id = None
    rules_path = None
    ego_maneuver_id = None
    nonego_maneuver_id = None
    try:
        opts, _ = getopt.getopt(
            argv, "ho:i:r:e:n:", ["ofile=", "intersection_id=", "rules_path=", "ego_maneuver_id=", "nonego_maneuver_id="])
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
        elif opt in ("-e", "--ego_maneuver_id"):
            ego_maneuver_id = int(arg)
        elif opt in ("-n", "--nonego_maneuver_id"):
            nonego_maneuver_id = int(arg)

    from scenario import Scenario

    scenario = Scenario()
    if intersection_id:
        scenario.intersection_id = intersection_id
    if rules_path:
        scenario.rules_path = rules_path
    if ego_maneuver_id:
        scenario.maneuver_id['ego'] = ego_maneuver_id

    from generator import Generator
    generator = Generator(map_path=scenario.map_path,
                          intersection_id=scenario.intersection_id,
                          rules_path=scenario.rules_path,
                          timestep=scenario.timestep,
                          maxSteps=scenario.maxSteps)
    scenario = generator.extend(scenario, maneuver_id=nonego_maneuver_id)

    import pickle
    with open(outputfile, 'wb') as outFile:
        pickle.dump(scenario, outFile)


if __name__ == "__main__":
    main(sys.argv[1:])
