import functools
import logging
import typing as t
from collections import namedtuple
from itertools import chain
from functools import partial
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import (NonlinearConstraint, minimize)
from tqdm import tqdm

from pyfmi import fmi

from mpc_optimization.fmu_source import FmuSource, ModelicaModelInfo, FmuType
from mpc_optimization.utils import ModelVariables, VariableType
from mpc_optimization.fmu_model import FmuMeAssimulo

StateConstraint = namedtuple('StateConstraint', ['var', 'lb', 'ub'])


class MPCOptimizer:
    def __init__(
            self,
            model_info: ModelicaModelInfo,
            state_variables: t.Dict[str, str],
            input_vec: t.List[str],  # FIXME: remove unused parameters
            output_vec: t.List[str],
            fmu_path: t.Optional[Path] = None,
            initial_parameters: t.Dict[str, t.Any] = dict(),
            points_per_sec: float = 1,
            verbose: bool = True):
        fmu_source = FmuSource.from_fmu(fmu_path) if fmu_path is not None else FmuSource.from_modelica(
            model_info, FmuType.ModelExchange)
        self.model = FmuMeAssimulo(fmu_source, initial_parameters, state_variables, verbose=verbose)

        # TODO: add checking for variables existence
        self.state_variables: t.Dict[str, str] = state_variables

        self.state_vars = self.model.get_varible_names(VariableType.STATE)
        self.input_vars = self.model.get_varible_names(VariableType.INPUT)
        self.output_vars = self.model.get_varible_names(VariableType.OUTPUT)

        self.points_per_sec = points_per_sec

    def simulate(self, start: float, end: float, input_df: pd.DataFrame, full_run: bool = True):
        if full_run:
            self.model.reset(start=start)
        opts = dict()
        opts['ncp'] = int((end - start) * self.points_per_sec)

        def find_index(timepoint):
            return np.argmin(np.abs(input_df.index - timepoint))

        res = self.model.simulate(start_time=start,
                                  final_time=end,
                                  input=(self.input_vars,
                                         input_df.reset_index().values[find_index(start):find_index(end)]),
                                  options=opts)

        def df_for(variables: t.List[str]) -> pd.DataFrame:
            return pd.DataFrame(data=np.array([res[var] for var in variables]).T, columns=variables)

        return df_for(sorted(set(chain(self.input_vars, self.state_vars,
                                       self.output_vars)))), df_for(self.input_vars), df_for(self.output_vars)

    def optimize(
        self,
        start: float,
        end: float,
        control_horizon: int,
        objective_func: t.Callable[[int, pd.DataFrame, pd.DataFrame, pd.DataFrame], float],
        initial_guess: pd.DataFrame,
        bounds: t.Dict[str, t.Tuple[float, float]] = dict(),
        constraints: t.List[StateConstraint] = [],
        simulate_horizon: int = 0,
        step: float = 1,
        iteration_callbacks: t.List[t.Callable[[int, pd.DataFrame], None]] = [],
        early_stopping_funcs: t.List[t.Callable[[int, ModelVariables], bool]] = []
    ) -> pd.DataFrame:
        step_state: t.Optional[np.ndarray] = None
        next_x0: t.Optional[np.ndarray] = None
        input_df = initial_guess.copy()

        if simulate_horizon is None:
            simulate_horizon = control_horizon

        for step_num, st in enumerate(tqdm(np.arange(start, end, step))):

            @functools.lru_cache
            def simulate_to_horizon(st, u) -> t.Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
                # input_df.iloc[input_df.index >= st] = u  # * C_rate_per_second
                for k, u_step in enumerate(u):
                    input_df.iloc[(input_df.index >= st + k * step) & (input_df.index < st +
                                                                       (k + 1) * step)] = u_step  # * C_rate_per_second
                # self._reset(start=st, control_df=input_df)
                self.model.reset(start=st, state=step_state)
                state, inputs, output = self.simulate(st, st + step * simulate_horizon, input_df, full_run=False)
                return state, inputs, output

            def sim_function(u):
                try:
                    state, inputs, output = simulate_to_horizon(st, tuple(u))
                    return objective_func(step_num, state, inputs, output)
                except fmi.FMUException as e:
                    logging.warning(e)
                    logging.warning(f'Simulation failed during opmization with inputs: {u}')
                    return 1000000000000

            def get_last_value(u, var: str):
                try:
                    _, _, outputs = simulate_to_horizon(st, tuple(u))
                    val = outputs[var].values[-1]
                except fmi.FMUException:
                    logging.warning(f'Simulation failed during opmization with inputs: {u}')
                    return np.nan
                return val

            scipy_constraints = [
                NonlinearConstraint(partial(get_last_value, var=cons.var), cons.lb, cons.ub) for cons in constraints
            ]

            def get_random_input(control_horizon: int):
                return np.random.uniform(low=bounds['I_req'][0], high=bounds['I_req'][1], size=(control_horizon))

            optim = minimize(sim_function,
                             x0=get_random_input(control_horizon) if next_x0 is None else next_x0,
                             method='SLSQP',
                             bounds=list(bounds.values()) * control_horizon,
                             args=(),
                             constraints=scipy_constraints,
                             options={
                                 'maxiter': 30,
                                 'ftol': 1e-08
                             })
            next_x0 = np.roll(optim.x, -1)
            next_x0[control_horizon - 1] = 0
            input_df.iloc[(input_df.index >= st)
                          & (input_df.index <= min(st + step, end))] = optim.x[0]  # * C_rate_per_second

            self.model.reset(start=st, state=step_state)

            state, _, _ = self.simulate(st, min(st + step, end), input_df, full_run=False)
            for callback in iteration_callbacks:
                callback(step_num, state)

            last_state = dict(zip(state.iloc[-1].index, state.iloc[-1].values))

            if any(func(step_num, last_state) for func in early_stopping_funcs):
                logging.info('Stopped optimization due to early stopping')
                return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]

            step_state = self.model.get_state()

        return input_df[:np.argmin(np.abs(input_df.index - (st + step)))]


if __name__ == '__main__':
    model: fmi.FMUModelCS2 = load_fmu(str(
        FmuSource.from_modelica(
            ModelicaModelInfo(Path('/home/developer/modelica/BatteryWithFullCycle.mo'),
                              'BatteryMPC.BatteryWithFullCycle')).fmu_path),
                                      log_level=6)
