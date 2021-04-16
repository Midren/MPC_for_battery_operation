import typing as t
from pathlib import Path

import numpy as np
import pandas as pd
from pyfmi import fmi, load_fmu
from pyfmi.fmi_algorithm_drivers import FMIResult
from tqdm import tqdm

import scipy
# from scipy.optimize import minimize_scalar

import os
import sys


class HiddenPrints:
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.close()
        sys.stdout = self._original_stdout


class FmuSource:
    def __init__(self, fmu_path):
        self.fmu_path: Path = fmu_path

    @classmethod
    def from_modelica(cls, model_name: str, model_location: Path):
        fmu_path = model_name + model_location
        # fmu_path = compile_fmu(model_name,
        #                        model_location,
        #                        target='cs',
        #                        version="2.0",
        #                        compiler_log_level='warning',  # 'info', 'warning',
        #                        compiler_options={"generate_html_diagnostics": True,
        #                                           "nle_solver_tol_factor": 1e-2})  # 1e-2 is the default
        return FmuSource(Path(fmu_path))

    @classmethod
    def from_fmu(cls, fmu_path: Path):
        return FmuSource(fmu_path)


class MPCOptimizer:
    def __init__(self, fmu: FmuSource, state_variables: t.List[str], input_vec: t.List[str], horizon_num: int, mpc_step: float):
        self.model: fmi.FMUModelCS2 = load_fmu(str(fmu.fmu_path), log_level=4)
        self.state_variables: t.List[str] = state_variables
        self.input_vars: t.List[str] = input_vec
        self.checkpoint: FMIResult
        self.horizon: int = horizon_num
        self.step: float = mpc_step

    def simulate(self, start: float, end: float, input_df: pd.DataFrame, save_all: bool = False):
        if start == 0:
            self._reset(0, None)
        opts = self.model.simulate_options()
        opts['ncp'] = 50
        opts['initialize'] = False

        res = self.model.simulate(start_time=start,
                                  final_time=end,
                                  input=(self.input_vars, input_df.reset_index().values[start:end]),
                                  options=opts)
        self.checkpoint = res
        if save_all:
            states = pd.DataFrame(data=np.array([res[i] for i in self.state_variables]).T, columns=self.state_variables)
            return states
        return {i: res.final(i) for i in self.state_variables}

    def _reinit_checkpoint(self, state):
        for state_var, val in state.items():
            if state_var == 'time':
                continue
            self.model.set(state_var, val)

    def _reset(self, time, state):
        self.model.reset()
        self.model.setup_experiment(start_time=time)
        self.model.initialize()
        if time != 0:
            self._reinit_checkpoint(state)

    def optimize(self, start: float, end: float, initial_guess: pd.DataFrame, objective_func: t.Callable[[t.Dict[str, t.Any]], float]):
        last_state: t.Optional[t.Dict[str, t.Any]] = None
        for st in tqdm(np.arange(start, end, self.step)):
            def sim_function(u, final=False):
                initial_guess.iloc[(initial_guess.index >= start) & (initial_guess.index <= end)] = u
                self._reset(st, last_state)
                state = self.simulate(st, min(st + self.step * self.horizon, end), initial_guess)
                if final:
                    return state
                return objective_func(state)

            with HiddenPrints():
                optim = scipy.optimize.minimize_scalar(sim_function, bounds=(-0.05, 0.05), args=(), method='bounded', options=None)
                last_state = sim_function(optim.x, final=True)
        return initial_guess
