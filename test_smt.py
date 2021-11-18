from pysmt.logics import QF_NRA
from pysmt.shortcuts import Real, get_env, And, Solver, Symbol, get_model, Equals
from pysmt.typing import REAL
import pysmt
import fractions

# solver_name = "z3-binary"
# path = ["/home/ak/Downloads/z3-4.8.10-x64-ubuntu-18.04/bin/z3", "-in", "-smt2"]

solver_name = "mathsat-binary"
path = ["/home/ak/Downloads/mathsat-5.6.6-linux-x86_64/bin/mathsat"]

# solver_name = "yices-binary"
# path = ["yices-smt2"]

# solver_name = "cvc4-binary"
# path = ["/home/ak/Downloads/cvc4-1.8-x86_64-linux-opt",
#         "--lang=smt2", "--produce-models", "--no-interactive-prompt"]

# solver_name = "smtrat-binary"
# path = ["/home/ak/Downloads/smtrat/build/smtrat-shared", "-"]


logics = [QF_NRA]

env = get_env()

# Add the solver to the environment
env.factory.add_generic_solver(solver_name, path, logics)

r, t = Symbol("r", REAL), Symbol("t", REAL)

constraints = [Equals(r, Real(2)), Equals(r*t, Real(1))]

with Solver(name=solver_name, logic=QF_NRA):
    m = get_model(And(constraints))
    val = m.get_py_value(t)
    if isinstance(m.get_py_value(t), fractions.Fraction):
        print(val.numerator/val.denominator)
