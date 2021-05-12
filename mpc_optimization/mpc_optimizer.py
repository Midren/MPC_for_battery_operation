import logging
import typing as t
from itertools import chain
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from pyfmi import fmi, load_fmu
from pyfmi.common.io import VariableNotFoundError
from pyfmi.fmi_algorithm_drivers import FMICSAlg, FMIResult
from scipy.optimize import minimize_scalar, least_squares
from tqdm import tqdm

from .fmi_cs_alg_progressbar import FMICSAlgWithProgressBar
from .fmu_source import FmuSource, ModelicaModelInfo
from .linearization import get_linear_model_matrices, linearize_model
from .utils import ModelVariables, VariableType

from .battery_model_constants import C_rate_per_second


class MPCOptimizer:
    def __init__(self,
                 model_info: ModelicaModelInfo,
                 state_variables: t.List[str],
                 input_vec: t.List[str],
                 output_vec: t.List[str],
                 horizon_num: int,
                 fmu_path: t.Optional[FmuSource] = None,
                 initial_parameters: t.Dict[str, t.Any] = dict(),
                 points_per_sec: float = 1):
        if fmu_path is not None:
            try:
                self.model: fmi.FMUModelCS2 = load_fmu(str(FmuSource.from_fmu(fmu_path).fmu_path),
                                                       log_level=6)
            except:
                self.model: fmi.FMUModelCS2 = load_fmu(str(
                    FmuSource.from_modelica(model_info).fmu_path),
                                                       log_level=4)
        else:
            self.model: fmi.FMUModelCS2 = load_fmu(str(
                FmuSource.from_modelica(model_info).fmu_path),
                                                   log_level=4)
        self.model.set_log_level(6)
        self.model.set_max_log_size(2073741824)  # = 2*1024^3 (about 2GB)

        # FMU exported from OpenModelica doesn't estimate from time 0,
        # so simulation from 0 to 0 helps
        self._set_vars(initial_parameters)
        opts = self.model.simulate_options()
        opts['silent_mode'] = True
        self.model.simulate(0, 0, options=opts)

        # TODO: add checking for variables existence
        # self.state_variables: t.List[str] = state_variables
        self.state_variables = [
            'time', *self.model.get_model_variables().keys()
            # filter='[!_]*').keys()
        ]
        self.input_vars: t.List[str] = input_vec
        self.output_vars: t.List[str] = output_vec
        self.horizon: int = horizon_num
        self.points_per_sec = points_per_sec

        all_parameters = list(self.model.get_model_variables(variability=1, filter='[!_]*').keys())
        for param in initial_parameters:
            if param not in all_parameters:
                raise ValueError("Variable is not model parameter:", param)
        self.initial_parameters = initial_parameters

        self.initial_state = {var: self.model.get(var) if var != 'time' else -1 for var in chain(self.state_variables, self.input_vars, self.output_vars)}

    def simulate(self,
                 start: float,
                 end: float,
                 input_df: pd.DataFrame,
                 save_all: bool = False,
                 verbose=True,
                 full_run: bool = True):
        if full_run:
            self._reset(start=start, control_df=input_df)
        opts = self.model.simulate_options()
        if start != end:
            opts['ncp'] = int((end - start) * self.points_per_sec)
        opts['initialize'] = False
        opts['silent_mode'] = True
        # opts["logging"] = True

        def find_index(timepoint):
            return np.argmin(np.abs(input_df.index - timepoint))

        res = self.model.simulate(start_time=start,
                                  final_time=end,
                                  input=(self.input_vars, input_df.reset_index().values[find_index(start):find_index(end)]),
                                  options=opts,
                                  algorithm=FMICSAlgWithProgressBar if verbose else FMICSAlg)

        def is_variable(var):
            try:
                return res.is_variable(var)
            except (KeyError, VariableNotFoundError):
                return False

        self.state_variables = list(filter(is_variable, self.state_variables))

        if save_all:
            states = pd.DataFrame(data=np.array([
                res[i]
                for i in sorted(set(chain(self.state_variables, self.input_vars, self.output_vars)))
            ]).T,
                                  columns=sorted(
                                      set(
                                          chain(self.state_variables, self.input_vars,
                                                self.output_vars))))
            return states

        def df_for(vars: t.List[str]) -> pd.DataFrame:
            return pd.DataFrame(data=np.array([res[var] for var in vars]).T, columns=vars)

        return df_for(self.state_variables), df_for(self.input_vars), df_for(self.output_vars)

    def _set_vars(self, state):
        for state_var, val in state.items():
            if state_var == 'time':
                continue
            self.model.set(state_var, val)

    def _reset(self,
               start: float,
               state: t.Optional[ModelVariables] = None,
               control_df: t.Optional[pd.DataFrame] = None):
        self.model.reset()

        self._set_vars(self.initial_parameters)

        # Currently after re-setting state variables
        # model simulate doesn't show the same behavior
        if state is not None:
            self.model.setup_experiment(start_time=start)
            self.model.initialize()
            self._set_vars(state)

            for var, val in self.initial_state.items():
                self.model.set(var, val)

        if control_df is not None:
            self.model.setup_experiment(start_time=0)
            self.model.initialize()

            if start > 0:
                self.simulate(start=0, end=start, input_df=control_df, verbose=False, full_run=False)

    def optimize(self,
                 start: float,
                 end: float,
                 initial_guess: pd.DataFrame,
                 objective_func: t.Callable[[ModelVariables, ModelVariables, ModelVariables],
                                            float],
                 bounds: t.Dict[str, t.Tuple[float, float]] = {},
                 step: float = 1,
                 iteration_callbacks: t.List[t.Callable[[int, pd.DataFrame], None]] = [],
                 early_stopping_funcs: t.List[t.Callable[[int, ModelVariables], bool]] = []):
        last_state: ModelVariables
        input_df = initial_guess.copy()

        for step_num, st in enumerate(tqdm(np.arange(start, end, step))):
            simulation_cache = dict()

            def sim_function(u, self):
                # input_df.iloc[(input_df.index >= st) & (input_df.index < st + step)] = u * C_rate_per_second
                input_df.iloc[input_df.index >= st] = u * C_rate_per_second
                self._reset(start=st, control_df=input_df)
                #                            state=last_state)
                try:
                    state, input, output = self.simulate(st,
                                                         st + step * self.horizon,
                                                         input_df,
                                                         verbose=False,
                                                         full_run=False)
                    simulation_cache[u] = (state, input, output)
                    return objective_func(step_num, state, input, output)
                except fmi.FMUException:
                    logging.warn('Simulation failed during opmization')
                    return 1000000000

            optim = least_squares(sim_function,
                                  bounds=bounds[self.input_vars[0]],
                                  args=(self),
                                  method='bounded',
                                  options={'xatol': 1e-7})
            input_df.iloc[(input_df.index >= st)
                          & (input_df.index <= min(st + step, end))] = optim.x * C_rate_per_second

            self._reset(start=st, control_df=input_df)
            #                            state=last_state)
            state, input, output = self.simulate(st,
                                                 min(st + step, end),
                                                 input_df,
                                                 verbose=False,
                                                 full_run=False)
            for callback in iteration_callbacks:
                callback(step_num, state)

            last_state = dict(zip(state.iloc[-1].index, state.iloc[-1].values))

            if any([func(step_num, last_state) for func in early_stopping_funcs]):
                logging.info('Stopped optimization due to early stopping')
                return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]

        return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]
