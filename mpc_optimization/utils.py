import os
import sys
import typing as t
from enum import Enum, unique

ModelVariables = t.Dict[str, t.Any]


@unique
class VariableType(Enum):
    STATE = 0
    INPUT = 1
    OUTPUT = 2

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
