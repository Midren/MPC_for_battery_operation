import itertools

import pandas as pd
import neptune.new as neptune

from mpc_optimization.battery_model_constants import State_vars_aliases_dict
from mpc_optimization.mpc_optimizer import ModelVariables


def visualize_output(step_num: int, states: pd.DataFrame, run: neptune.Run):
    for _, state in itertools.islice(states.iterrows(), len(states) - 1):
        run['time'].log(state['time'])
        run['state/SoC'].log(state['SoC'])
        run['state/SoH'].log(state['SoH'])
        run['input/I_req'].log(state['I_req'])


def stop_if_discharged(step_num: int, state: ModelVariables):
    if state[State_vars_aliases_dict['SoC']] <= 0.01:
        return True
    return False


def stop_if_charged(step_num: int, state: ModelVariables):
    if state[State_vars_aliases_dict['SoC']] >= 0.99:
        return True
    return False
