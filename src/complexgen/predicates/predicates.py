min_perceptible_time = 10  # frames
import clingo

class TemporalContext:
  def __init__(self, sym2val):
    self.sym2val = sym2val

  def lessThan(self, S, T):
    lt = self.sym2val[S.name] + min_perceptible_time < self.sym2val[T.name]
    return clingo.Number(1) if lt else clingo.Number(0)

  def equal(self, S, T):
    eq = abs(self.sym2val[S.name] - self.sym2val[T.name]) < min_perceptible_time
    return clingo.Number(1) if eq else clingo.Number(0)
  
  # To generate unique time constants:
  def time(self, V, Pred, Tm, TM):
    t = V.name + Pred.name + Tm.name + TM.name
    return clingo.Function(t, [])