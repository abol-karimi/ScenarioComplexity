import clingo


class Solver():
    def __init__(self, maxtime):
        self.maxtime = maxtime
        self.__solution = None
        self.__ctl = clingo.Control()

    def load(self, path):
        self.__ctl.load(path)

    def add_atoms(self, atoms):
        program = ""
        for atom in atoms:
            program += f'{atom}.\n'
        self.__ctl.add("boundedtime", [self.maxtime], program)

    def solve(self):
        print("\nSolving...")
        self.__ctl.ground([("boundedtime", [self.maxtime])])
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
