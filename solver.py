import clingo


class Solver():
    def __init__(self, rules_path="uncontrolled-intersection.lp"):
        self.__solution = None
        self.__ctl = clingo.Control()
        self.__ctl.load(rules_path)

    def add_atoms(self, atoms):
        program = ""
        for atom in atoms:
            program += f'{atom}.\n'
        self.__ctl.add("base", [], program)

    def solve(self):
        print("\n Solving...")
        self.__ctl.ground([("base", [])])
        self.__ctl.solve(on_model=self.__on_model)

    def __on_model(self, model):
        for atom in model.symbols(atoms=True):
            if atom.match("violatesRightOf", 2):
                print(atom)
            if atom.match("mustYieldToForRuleAtTime", 4):
                print(atom)

    def get_solutions(self):
        return self.__solutions
