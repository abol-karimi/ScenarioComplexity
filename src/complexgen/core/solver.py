import clingo


class NoASPSolutionError(Exception):
    """Exception raised for errors in ASP solving.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, message):
        self.message = message


class ASPSolver():
    def __init__(self):
        self.__solution = None
        self.__ctl = clingo.Control()

    def load(self, path):
        self.__ctl.load(path)

    def add_atoms(self, atoms):
        program = ""
        for atom in atoms:
            program += f'{atom}.\n'
        self.__ctl.add("base", [], program)

    def solve(self):
        print("\nSolving...")
        self.__ctl.ground([("base", [])])
        result = self.__ctl.solve(on_model=self.__on_model, async_=False)

        if not result.satisfiable:
            message = f''
            f'Exhausted = {result.exhausted}\n'
            f'Interrupted = {result.interrupted}\n'
            f'Satisfiable = {result.satisfiable}\n'
            f'Unknown = {result.unknown}\n'
            f'Unsatisfiable = {result.unsatisfiable}'
            raise NoASPSolutionError(message)

        return self.__solution

    def __on_model(self, model):
        print('ASP model found!')
        self.__solution = model.symbols(atoms=True)
