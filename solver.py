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
        print("\nSolving...")
        self.__ctl.ground([("base", [])])
        self.__ctl.solve(on_model=self.__on_model,
                         on_finish=self.__on_finish, async_=False)

        return self.__solution

    def __on_model(self, model):
        print('Model found!')
        self.__solution = model.symbols(atoms=True)

    def __on_finish(self, result):
        if not result.satisfiable:
            print(f'Exhausted = {result.exhausted}')
            print(f'Interrupted = {result.interrupted}')
            print(f'Satisfiable = {result.satisfiable}')
            print(f'Unknown = {result.unknown}')
            print(f'Unsatisfiable = {result.unsatisfiable}')
