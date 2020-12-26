import clingo


class Solver():
    def __init__(self, rules_path="uncontrolled-intersection.lp"):
        self.__solution = set()
        self.__ctl = clingo.Control()
        self.__ctl.load(rules_path)

    def add_atoms(self, atoms):
        program = ""
        for atom in atoms:
            program += f'{atom}.\n'
        print('Adding atoms:')
        print(program)
        self.__ctl.add("base", [], program)

    def solve(self):
        print("\n Solving...")
        self.__ctl.ground([("base", [])])
        self.__ctl.solve(on_model=self.__on_model, on_finish=self.__on_finish)
        return self.__solution

    def __on_model(self, model):
        self.__solution = model
        self.__solution = set()
        for atom in model.symbols(atoms=True):
            if atom.match("violatesRightOf", 2):
                cars = [arg.name for arg in atom.arguments]
                self.__solution.add(tuple([cars[0], cars[1]]))
                print(atom)
            else:
                print(atom)

    def __on_finish(self, result):
        print("Exhausted = " + str(result.exhausted))
        print("Interrupted = " + str(result.interrupted))
        print("Satisfiable = " + str(result.satisfiable))
        print("Unknown = " + str(result.unknown))
        print("Unsatisfiable = " + str(result.unsatisfiable))
