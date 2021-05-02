import logging
import typing as t
from itertools import chain

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from pyfmi import fmi, load_fmu
from pyfmi.common.io import VariableNotFoundError
from pyfmi.fmi_algorithm_drivers import FMICSAlg, FMIResult
from scipy.optimize import minimize_scalar
from tqdm import tqdm

from .fmi_cs_alg_progressbar import FMICSAlgWithProgressBar
from .fmu_source import FmuSource

ModelVariables = t.Dict[str, t.Any]


class MPCOptimizer:
    def __init__(
            self,
            fmu: FmuSource,
            state_variables: t.List[str],
            input_vec: t.List[str],
            output_vec: t.List[str],
            # initial_parameters: t.Dict[str, t.Any],
            horizon_num: int,
            points_per_sec: float = 1):
        self.model: fmi.FMUModelCS2 = load_fmu(str(fmu.fmu_path), log_level=4)
        self.model.set_max_log_size(2073741824)  # = 2*1024^3 (about 2GB)
        # TODO: add checking for variables existence
        # self.state_variables: t.List[str] = state_variables
        self.state_variables = [
            'time', *self.model.get_model_variables().keys()
            # filter='[!_]*').keys()
        ]
        self.input_vars: t.List[str] = input_vec
        self.output_vars: t.List[str] = output_vec
        self.checkpoint: FMIResult
        self.horizon: int = horizon_num
        self.points_per_sec = points_per_sec

        self.initial_state = {var: self.model.get(var) for var in self.model.get_states_list()}

    def simulate(self,
                 start: float,
                 end: float,
                 input_df: pd.DataFrame,
                 save_all: bool = False,
                 verbose=True,
                 full_run: bool = True):
        if full_run:
            self._reset(start_time=start, control_df=input_df)
        opts = self.model.simulate_options()
        opts['ncp'] = int((end - start) * self.points_per_sec)
        opts['initialize'] = False
        opts['silent_mode'] = True
        # opts["logging"] = True
        self.model.set("_log_level", 4)

        res = self.model.simulate(start_time=start,
                                  final_time=end,
                                  input=(self.input_vars, input_df.reset_index().values[start:end]),
                                  options=opts,
                                  algorithm=FMICSAlgWithProgressBar if verbose else FMICSAlg)
        self.checkpoint = res
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

        def is_variable(var):
            try:
                return self.checkpoint.is_variable(var)
            except (KeyError, VariableNotFoundError):
                return False

        self.state_variables = list(filter(is_variable, self.state_variables))

        def df_for(vars: t.List[str]) -> pd.DataFrame:
            return pd.DataFrame(data=np.array([res[var] for var in vars]).T, columns=vars)

        return df_for(self.state_variables), df_for(self.input_vars), df_for(self.output_vars)

    def _reinit_state(self, state):
        for state_var, val in state.items():
            if state_var == 'time':
                continue
            self.model.set(state_var, val)

    def _reset(self, start_time, state=None, control_df=None):
        self.model.reset()

        # Currently after re-setting state variables
        # model simulate doesn't show the same behavior
        #
        if state is not None:
            self.model.setup_experiment(start_time=start_time)
            self.model.initialize()
            self._reinit_state(state)

            for var, val in self.initial_state.items():
                self.model.set(var, val)

        if control_df is not None:
            self.model.setup_experiment(start_time=0)
            self.model.initialize()

            if start_time > 0:
                self.simulate(0, start_time, input_df=control_df, verbose=False)

    def optimize(
            self,
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
        for step_num, st in enumerate(tqdm(np.arange(start + 1, end, step))):
            simulation_cache = dict()

            def sim_function(u, self, last_state):
                input_df.iloc[(input_df.index >= st)
                                   & (input_df.index < st + step)] = u
                self._reset(start_time=st, control_df=input_df)
                #                            state=last_state)
                try:
                    state, input, output = self.simulate(st,
                                                         st + step * self.horizon,
                                                         input_df,
                                                         verbose=False,
                                                         full_run=False)
                    simulation_cache[u] = (state, input, output)
                    return objective_func(state, input, output)
                except:
                    return 1000000000

            optim = minimize_scalar(sim_function,
                                    bounds=bounds[self.input_vars[0]],
                                    args=(self, last_state),
                                    method='bounded',
                                    options={'xatol': 1e-7})
            input_df.iloc[(input_df.index >= st)
                               & (input_df.index <= min(st + step, end))] = optim.x

            self._reset(start_time=st, control_df=input_df)
            #                            state=last_state)
            state, input, output = self.simulate(st,
                                                 min(st + step, end),
                                                 input_df,
                                                 verbose=False,
                                                 full_run=False)
            for callback in iteration_callbacks:
                callback(step_num, state)

            last_state = dict(zip(state.iloc[-1].index, state.iloc[-1].values))
            self.initial_state = {var: self.model.get(var) for var in self.model.get_states_list()}

            if any([func(step_num, last_state) for func in early_stopping_funcs]):
                logging.info('Stopped optimization due to early stopping')
                return input_df

        return input_df
