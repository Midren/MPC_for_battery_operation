import logging
import typing as t
from enum import Enum, unique
from itertools import chain
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from pyfmi import fmi, load_fmu
from pyfmi.common.io import VariableNotFoundError
from pyfmi.fmi_algorithm_drivers import FMICSAlg, FMIResult
from scipy.optimize import least_squares
from tqdm import tqdm

from .fmi_cs_alg_progressbar import FMICSAlgWithProgressBar
from .fmu_source import FmuSource, ModelicaModelInfo
from .linearization import get_linear_model_matrices, linearize_model
from .utils import ModelVariables, VariableType


class MPCOptimizerWithLinearization:
    def __init__(self,
                 model_info: ModelicaModelInfo,
                 horizon_num: int,
                 initial_parameters: ModelVariables = dict(),
                 initial_outputs: ModelVariables = dict(),
                 points_per_sec: float = 1):
        self.model_info = model_info
        # Add initial parameters setting before linearization
        self.model: fmi.FMUModelCS2 = linearize_model(self.model_info, initial_parameters)

        self.state_variables: t.List[str] = self._get_variables(VariableType.STATE)
        self.input_vars: t.List[str] = self._get_variables(VariableType.INPUT)
        self.output_vars: t.List[str] = self._get_variables(VariableType.OUTPUT)
        self.d = {
            self._get_linearized_var(var, VariableType.OUTPUT): val
            for var, val in initial_outputs.items()
        }

        incorrect_outputs = list(filter(lambda x: x not in self.output_vars, self.d))
        if len(incorrect_outputs):
            raise ValueError("Next variables are not output ones:",
                             list(map(lambda x: x[3:-1], incorrect_outputs)))

        self.horizon: int = horizon_num
        self.points_per_sec = points_per_sec

        # Add checking for inputs via OMPython as linearizated model doesn't have
        # parameters
        self.initial_parameters = initial_parameters

    def simulate(self,
                 start: float,
                 end: float,
                 input_df: pd.DataFrame,
                 save_all: bool = False,
                 verbose=True):
        input_df = self._prepare_input_for_linear(input_df, start, end)
        opts = self.model.simulate_options()
        opts['ncp'] = int((end - start) * self.points_per_sec)
        opts['initialize'] = False
        opts['silent_mode'] = True
        # opts["logging"] = True
        # self.model.set("_log_level", 4)

        input_df.rename(columns={'I_req': "'u_I_req'"}, inplace=True)
        res = self.model.simulate(start_time=0,
                                  final_time=end - start,
                                  input=(self.input_vars, input_df.reset_index().values),
                                  options=opts,
                                  algorithm=FMICSAlgWithProgressBar if verbose else FMICSAlg)

        def df_for(vars: t.List[str]) -> pd.DataFrame:
            stripped_vars = list(map(lambda x: x.strip()[3:-1] if x.startswith("'") else x, vars))
            return pd.DataFrame(data=np.array([res[var] for var in vars]).T, columns=stripped_vars)

        def add_start_values(variables_df: pd.DataFrame):
            for var, val in self.d.items():
                variables_df[var[3:-1]] += val
            return variables_df

        return df_for(["time", *self.state_variables
                       ]), df_for(self.input_vars), add_start_values(df_for(self.output_vars))

    def _set_vars(self, state):
        for state_var, val in state.items():
            if state_var == 'time':
                continue
            self.model.set(state_var, val)

    def _reset(self):
        # fmu_path = FmuSource.from_modelica(
        # ModelicaModelInfo(Path("linearized_model.mo"), "linearized_model")).fmu_path
        # fmu_path = FmuSource.from_fmu(Path('linearized_model.fmu')).fmu_path
        # self.model = load_fmu(str(fmu_path))
        self.model.reset()

        # FMU exported from OpenModelica doesn't estimate from time 0,
        # so simulation from 0 to 0 helps
        opts = self.model.simulate_options()
        opts['silent_mode'] = True
        self.model.simulate(0, 0, options=opts)

    @staticmethod
    def _form_sub_frame(control_df: pd.DataFrame, start: float, end: float):
        def find_index(timepoint):
            return np.argmin(np.abs(control_df.index - timepoint))

        return control_df.iloc[find_index(start):find_index(end)]

    @staticmethod
    def _shift_time_to_zero(control_df: pd.DataFrame):
        new_df = control_df.copy().reset_index(drop=True)
        new_df['time'] = control_df.index - control_df.index[0]
        new_df.set_index('time', inplace=True)
        return new_df

    @staticmethod
    def _prepare_input_for_linear(control_df: pd.DataFrame, start: float, end: float):
        return MPCOptimizerWithLinearization._shift_time_to_zero(
            MPCOptimizerWithLinearization._form_sub_frame(control_df, start, end))

    def _get_variables(self, variable_type: VariableType):
        prefix = variable_type.linear_prefix()
        return list(self.model.get_model_variables(filter=f"'{prefix}*").keys())

    def _get_linearized_var(self, var_name: str, variable_type: VariableType):
        prefix = variable_type.linear_prefix()
        return f"'{prefix}{var_name}'"

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
        last_state: t.Optional[ModelVariables] = None
        input_df = initial_guess.copy()
        for step_num, st in enumerate(tqdm(np.arange(start, end, step))):
            self.model = linearize_model(
                self.model_info, self.initial_parameters,
                self._prepare_input_for_linear(input_df, 0, st) if st > 0 else None)
            simulation_cache = dict()

            def sim_function(u, self, last_state):
                input_df.iloc[input_df.index >= st] = u
                linear_input_df = MPCOptimizerWithLinearization._prepare_input_for_linear(
                    input_df, st, st + step * self.horizon)
                self._reset()
                try:
                    state, input, output = self.simulate(0,
                                                         step * self.horizon,
                                                         linear_input_df,
                                                         verbose=False)
                    simulation_cache[u] = (state, input, output)
                    J = objective_func(step_num, state, input, output)
                    return J
                except fmi.FMUException:
                    return 1000000000

            optim = least_squares(sim_function,
                                  bounds=bounds[self.input_vars[0][3:-1]],
                                  args=(self, last_state),
                                  method='trf')
            input_df.iloc[(input_df.index >= st)
                          & (input_df.index <= min(st + step, end))] = optim.x

            self._reset()

            linear_input_df = MPCOptimizerWithLinearization._prepare_input_for_linear(
                input_df, 0,
                min(st + step, end) - st)
            state, input, output = self.simulate(0,
                                                 min(st + step, end) - st,
                                                 linear_input_df,
                                                 verbose=False)
            all_variables = pd.concat([state, input, output], axis=1, join='inner')
            for callback in iteration_callbacks:
                callback(step_num, all_variables)

            last_state = dict(zip(all_variables.iloc[-1].index, all_variables.iloc[-1].values))
            # for var in self.d:
                # self.d[var] = last_state[var[3:-1]]
            # print(self.d)

            if any([func(step_num, last_state) for func in early_stopping_funcs]):
                logging.info('Stopped optimization due to early stopping')
                return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]

        return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]

    def get_matrices(self):
        return get_linear_model_matrices(self.model)
