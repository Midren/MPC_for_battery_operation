import typing as t
from pathlib import Path
import itertools

import pandas as pd
import numpy as np
from pyfmi import fmi, FMUModelME2, load_fmu
from scipy.integrate import solve_ivp

from mpc_optimization.utils import ModelVariables
from mpc_optimization.fmu_source import FmuSource


class Fmu:
    def __init__(self, fmu_source: FmuSource, initial_parameters: ModelVariables = dict()):
        self.model: FMUModelME2 = load_fmu(str(fmu_source.fmu_path), log_level=4)
        self.model.set_max_log_size(2073741824)  # = 2*1024^3 (about 2GB)
        self.initial_params = initial_parameters

        self.reset(0)

    def reset(self, start: float):
        ...

    def set_params(self, state):
        self.model.initialize()
        for state_var, val in state.items():
            if str(state_var) == 'time':
                continue
            self.model.set(state_var, val)


class FmuME(Fmu):
    def __init__(self,
                 fmu_source: FmuSource,
                 initial_parameters: ModelVariables = dict(),
                 state_variables: ModelVariables = dict()):
        self.state_variables = state_variables
        super().__init__(fmu_source, initial_parameters)

    def reset(self, start: float, state: ModelVariables = dict()):
        self.model.reset()

        self.model.setup_experiment(start_time=start)
        self.set_params(self.initial_params)
        for read_val, param in self.state_variables.items():
            self.model.set(param, state[read_val])
            # TODO: fix to set only once
            self.model.set('start_UIC', True)

        e_info = self.model.get_event_info()
        e_info.newDiscreteStatesNeeded = True
        # Event iteration
        while e_info.newDiscreteStatesNeeded:
            self.model.enter_event_mode()
            self.model.event_update()
            e_info = self.model.get_event_info()

        self.model.enter_continuous_time_mode()

        self.input_names = list(self.model.get_model_variables(causality=fmi.FMI2_INPUT, include_alias=False).keys())
        self.state_names = list(self.model.get_model_variables(causality=fmi.FMI2_LOCAL, include_alias=False).keys())
        self.output_names = list(self.model.get_model_variables(causality=fmi.FMI2_OUTPUT, include_alias=False).keys())
        self.model_vars_idx = np.array([self.model.get_variable_valueref(k) for k in itertools.chain(self.input_names, self.state_names, self.output_names)])

    def _simulate(self, start_time, final_time, points_num):
        x0 = self.model.continuous_states

        rtol, atol = self.model.get_tolerances()
        # t_eval = np.linspace(start_time, final_time, points_num)
        sol_out = solve_ivp(self._deriv, (start_time, final_time),
                            x0,
                            method='LSODA',
                            jac=self._jacobian,
                            rtol=rtol,
                            atol=atol,
                            # t_eval=t_eval,
                            )

        self.model.continuous_states = sol_out.y[:, -1].copy(order='C')
        return self.model.get_real(self.model_vars_idx)

    def form_res(self, states: t.Sequence[np.ndarray]):
        return {name: val for name, val in zip(itertools.chain(self.input_names, self.state_names, self.output_names), np.vstack(states).T)}

    def simulate(self, start_time, final_time, input=None, options=None):
        points_num = options['ncp']
        states = []

        st = start_time
        for input_vals in input[1]:
            if input_vals[0] > 0:
                state = self._simulate(st, input_vals[0], points_num)
                states.append(state)
            for input_name, input_val in zip(input[0], input_vals[1:]):
                self.model.set(input_name, input_val)
            st = input_vals[0]
        state = self._simulate(st, final_time, points_num)
        states.append(state)

        return self.form_res(states)

    def _deriv(self, t: float, x: np.ndarray) -> np.ndarray:
        self.model.time = t
        self.model.continuous_states = x.copy(order='C')
        return self.model.get_derivatives()

    def _jacobian(self, t: float, x: np.ndarray):
        # time and state are updated during _deriv function call
        state_values = [state.value_reference for state in self.model.get_states_list().values()]
        deriv_values = [der.value_reference for der in self.model.get_derivatives_list().values()]
        jac = np.identity(len(deriv_values))
        return np.apply_along_axis(lambda col: self.model.get_directional_derivative(state_values, deriv_values, col), 0, jac)


class FmuCS(Fmu):
    def reset(self, start: float, control_df: pd.DataFrame = None):
        self.model.reset()
        self.model.setup_experiment(start_time=0)
        self.set_params(self.initial_params)

        # Currently after re-setting state variables
        # model simulate doesn't show the same behavior
        if start > 0:
            self.model.simulate(start=0, end=start, input_df=control_df, verbose=False, full_run=False)


if __name__ == "__main__":
    from mpc_optimization.fmu_source import ModelicaModelInfo
    source = FmuSource.from_modelica(
        ModelicaModelInfo(Path('/home/developer/modelica/BatteryWithFullCycle.mo'), 'BatteryMPC.BatteryWithFullCycle'))
    model = FmuME(source)

    start_time = 0
    final_time = 30 * 60
    control_df = pd.DataFrame({"Time": range(start_time, final_time)})
    control_df['I_req'] = -1e-4
    control_df.set_index('Time', inplace=True)

    def find_index(timepoint):
        return np.argmin(np.abs(control_df.index - timepoint))

    input = (['I_req'], control_df.reset_index().values[find_index(start_time):find_index(final_time)+1])

    states = model.simulate(start_time, final_time, input=input, options={'ncp': 100})
    print(states)
