import os
import sys
import typing as t
from enum import Enum, unique

import cProfile
import pstats
import io

ModelVariables = t.Dict[str, t.Any]

def profileit(func):
    def wrapper(*args, **kwargs):
        datafn = func.__name__ + ".profile" # Name the data file sensibly
        prof = cProfile.Profile()
        retval = prof.runcall(func, *args, **kwargs)
        s = io.StringIO()
        sortby = 'cumulative'
        ps = pstats.Stats(prof, stream=s).sort_stats(sortby)
        ps.print_stats()
        with open(datafn, 'w') as perf_file:
            perf_file.write(s.getvalue())
        return retval

    return wrapper

@unique
class VariableType(Enum):
    INPUT = 0
    OUTPUT = 1
    STATE = 2

    def linear_prefix(self):
        if self.value == VariableType.STATE.value:
            return "x_"
        elif self.value == VariableType.INPUT.value:
            return "u_"
        elif self.value == VariableType.OUTPUT.value:
            return "y_"


class HiddenPrints:
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.close()
        sys.stdout = self._original_stdout
