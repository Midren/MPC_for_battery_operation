import typing as t
from pathlib import Path
import itertools

import pandas as pd
import numpy as np
from pyfmi import fmi, FMUModelME2, load_fmu
from pyfmi.fmi_algorithm_drivers import FMICSAlg, FMIResult
from scipy.integrate import solve_ivp

from mpc_optimization.fmi_cs_alg_progressbar import FMICSAlgWithProgressBar
from mpc_optimization.utils import ModelVariables, VariableType
from mpc_optimization.fmu_source import FmuSource, FmuType

from time import perf_counter
from contextlib import contextmanager


@contextmanager
def catchtime() -> float:
    start = perf_counter()
    yield lambda: perf_counter() - start


class Fmu:
    def __init__(self, fmu_source: FmuSource, initial_parameters: ModelVariables = {}, verbose: bool = False):
        self.model: FMUModelME2 = load_fmu(str(fmu_source.fmu_path), log_level=0)
        self.model.set_max_log_size(2 * 1024**3)

        all_parameters = list(
            self.model.get_model_variables(causality=fmi.FMI2_PARAMETER, include_alias=False, filter='[!_]*').keys())
        # TODO: rewrite using filter
        for param in initial_parameters:
            if param not in all_parameters:
                raise ValueError("Variable is not model parameter:", param)
        self.initial_params = initial_parameters
        self.verbose = verbose

        self.input_names = list(self.model.get_model_variables(causality=fmi.FMI2_INPUT, include_alias=False).keys())
        self.state_names = [
            'time', *list(self.model.get_model_variables(causality=fmi.FMI2_LOCAL, include_alias=False).keys())
        ]
        self.output_names = list(self.model.get_model_variables(causality=fmi.FMI2_OUTPUT, include_alias=False).keys())

        self.reset(0)

    def reset(self, start: float):
        ...

    def set_params(self, state):
        for state_var, val in state.items():
            if str(state_var) == 'time':
                continue
            self.model.set(state_var, val)

    def get_varible_names(self, var_type: VariableType) -> t.List[str]:
        causality = var_type.fmi_constant()
        var_names = list(self.model.get_model_variables(include_alias=False, causality=causality).keys())
        # FIXME: temporary fix
        # Add possibility to save/use any variable during optimization
        if var_type == VariableType.STATE:
            var_names = ['time'] + var_names
        return var_names


class FmuMeAbstract(Fmu):
    def __init__(self,
                 fmu_source: FmuSource,
                 initial_parameters: ModelVariables = dict(),
                 state_variables: ModelVariables = dict(),
                 verbose: bool = False):
        self.state_variables = state_variables
        super().__init__(fmu_source, initial_parameters, verbose)

        self.state_refs = [state.value_reference for state in self.model.get_states_list().values()]
        self.state_deriv_refs = [der.value_reference for der in self.model.get_derivatives_list().values()]

        self.output_refs = [output.value_reference for output in self.model.get_output_list().values()]

    def reset(self, start: float, state: t.Optional[np.ndarray] = None):
        self.model.reset()

        self.model.setup_experiment(start_time=start)
        self.model.initialize()

        if state is None:
            self.set_params(self.initial_params)
        else:
            self.model.time = start
            self.model.continuous_states = state

        e_info = self.model.get_event_info()
        e_info.newDiscreteStatesNeeded = True
        # Event iteration
        while e_info.newDiscreteStatesNeeded:
            self.model.enter_event_mode()
            self.model.event_update()
            e_info = self.model.get_event_info()

        self.model.enter_continuous_time_mode()

    def get_state(self):
        return self.model.continuous_states

    def simulate(self, start_time, final_time, input=None, options=None):
        raise NotImplementedError("Abstract class")


class FmuMeSciPy(FmuMeAbstract):
    def __init__(self,
                 fmu_source: FmuSource,
                 initial_parameters: ModelVariables = dict(),
                 state_variables: ModelVariables = dict(),
                 verbose: bool = False):
        super().__init__(fmu_source, initial_parameters, state_variables, verbose)
        self.state_names.remove('time')
        self.model_vars_idx = np.array([
            self.model.get_variable_valueref(k)
            for k in itertools.chain(self.input_names, self.state_names, self.output_names)
        ])

    def _deriv(self, t: float, x: np.ndarray) -> np.ndarray:
        self.model.time = t
        self.model.continuous_states = x.copy(order='C')
        res = self.model.completed_integrator_step()
        if res[0] == True:
            self.model.event_update()
        return self.model.get_derivatives()

    def _jacobian(self, t: float, x: np.ndarray):
        # time and state are updated during _deriv function call
        jac = np.identity(len(self.state_deriv_refs))
        jac = np.apply_along_axis(
            lambda col: self.model.get_directional_derivative(self.state_refs, self.state_deriv_refs, col), 0, jac)
        return jac

    def _simulate(self, start_time, final_time, points_num):
        x0 = self.model.continuous_states

        rtol, atol = self.model.get_tolerances()
        # t_eval = np.linspace(start_time, final_time, points_num)
        sol_out = solve_ivp(
            self._deriv,
            (start_time, final_time),
            x0,
            method='LSODA',
            jac=self._jacobian,
            rtol=rtol,
            atol=atol,
            # t_eval=t_eval,
        )

        self.model.continuous_states = sol_out.y[:, -1].copy(order='C')
        return self.model.get_real(self.model_vars_idx)

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
        self.model.setup_experiment(start_time)

        return self.form_res(states)

    def form_res(self, states: t.Sequence[np.ndarray]):
        return {
            name: val
            for name, val in zip(itertools.chain(self.input_names, self.state_names, self.output_names),
                                 np.vstack(states).T)
        }


class FmuMeAssimulo(FmuMeAbstract):
    def simulate(self, start_time, final_time, input=None, options=None):
        points_num = options['ncp']

        opts = self.model.simulate_options()
        opts['ncp'] = points_num
        opts['initialize'] = False
        # opts['solver'] = 'RodasODE'
        opts['solver'] = 'LSODAR'
        # opts['solver'] = 'Dopri5'
        opts[f'{opts["solver"]}_options']["verbosity"] = 50
        opts['logging'] = False
        res = self.model.simulate(start_time=start_time, final_time=final_time, input=input, options=opts)
        return self.form_res(res)

    def form_res(self, states: FMIResult):
        return {name: states[name] for name in itertools.chain(self.input_names, self.state_names, self.output_names)}


class FmuCS(Fmu):
    def reset(self, start: float, control_df: pd.DataFrame = None):
        self.model.reset()
        self.model.setup_experiment(start_time=0)
        self.set_params(self.initial_params)

        # Currently after re-setting state variables
        # model simulate doesn't show the same behavior
        if start > 0:
            if control_df is None:
                raise RuntimeError("No input for simulation")

            def find_index(timepoint):
                return np.argmin(np.abs(control_df.index - timepoint))

            self.simulate(start_time=0,
                          final_time=start,
                          input=(self.input_names, control_df.reset_index().values[find_index(0):find_index(start)]))

    def form_res(self, states: FMIResult):
        return {name: states[name] for name in itertools.chain(self.input_names, self.state_names, self.output_names)}

    def simulate(self, start_time, final_time, input, options=None):
        res = self.model.simulate(start_time=start_time,
                                  final_time=final_time,
                                  input=input,
                                  options=options,
                                  algorithm=FMICSAlgWithProgressBar if self.verbose else FMICSAlg)
        return self.form_res(res)


if __name__ == "__main__":
    from mpc_optimization.fmu_source import ModelicaModelInfo
    source = FmuSource.from_modelica(
        ModelicaModelInfo(Path('/home/developer/modelica/BatteryWithFullCycle.mo'), 'BatteryMPC.BatteryWithFullCycle'),
        FmuType.ModelExchange)
    # source = FmuSource.from_fmu(Path("/home/developer/python_libs/BatteryMPC.BatteryWithFullCycle.fmu"))
    model = FmuMeAssimulo(source, {'theveninBasedBattery.coulombSocCounter.SOC_init.k': 0.0})

    start_time = 0
    control_df = pd.DataFrame({"Time": [start_time]})
    final_time = 1 * 60 * 60
    control_df['I_req'] = -1e-4
    control_df.set_index('Time', inplace=True)

    def find_index(timepoint):
        return np.argmin(np.abs(control_df.index - timepoint))

    input = (['I_req'], control_df.reset_index().values[find_index(start_time):find_index(final_time) + 1])

    from tqdm import trange
    st = 0
    en = final_time // 100
    last_state = None
    for i in trange(100):
        model.reset(st, state=last_state)
        with catchtime() as t:
            states = model.simulate(st, en, input=input, options={'ncp': 100})
            last_state = model.get_state()
            st = en
            en += final_time // 100
            states = model.simulate(st, en, input=input, options={'ncp': 100})
